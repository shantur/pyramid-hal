/*
** Copyright (c) 2011-2013 The Linux Foundation. All rights reserved.
**
** Not a Contribution, Apache license notifications and license are retained
** for attribution purposes only.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

/*#error uncomment this for compiler test!*/

#define LOG_TAG "QCameraHWI_Rdi"
#include <utils/Log.h>
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "QCameraHAL.h"
#include "QCameraHWI.h"
#include <gralloc_priv.h>

#define UNLIKELY(exp) __builtin_expect(!!(exp), 0)
#define LIKELY(exp) __builtin_expect(!!(exp), 1)

/* QCameraHWI_Raw class implementation goes here*/
/* following code implement the RDI logic of this class*/

namespace android {

status_t   QCameraStream_Rdi::freeBufferRdi()
{
    int err = 0;
    status_t ret = NO_ERROR;

    ALOGE(" %s : E ", __FUNCTION__);
    mHalCamCtrl->mRdiMemoryLock.lock();
    mHalCamCtrl->releaseHeapMem(&mHalCamCtrl->mRdiMemory);
    memset(&mHalCamCtrl->mRdiMemory, 0, sizeof(mHalCamCtrl->mRdiMemory));
    mHalCamCtrl->mRdiMemoryLock.unlock();

    ALOGI(" %s : X ",__FUNCTION__);
    return NO_ERROR;
}
status_t QCameraStream_Rdi::updatePreviewMetadata(const void *yuvMetadata)
{
    memset(mHalCamCtrl->mFace, 0, sizeof(mHalCamCtrl->mFace));
    mHalCamCtrl->mMetadata.faces = mHalCamCtrl->mFace;
    mHalCamCtrl->mMetadata.number_of_faces = 0;

    //TODO: Fill out face information based on yuvMetadata

    mHalCamCtrl->mCallbackLock.lock();
    camera_data_callback pcb = mHalCamCtrl->mDataCb;
    mHalCamCtrl->mCallbackLock.unlock();

    if (pcb && (mHalCamCtrl->mMsgEnabled & CAMERA_MSG_PREVIEW_METADATA)) {
        ALOGE("%s: Face detection RIO callback", __func__);
        pcb(CAMERA_MSG_PREVIEW_METADATA, NULL, 0,
            &mHalCamCtrl->mMetadata, mHalCamCtrl->mCallbackCookie);
    }

    return NO_ERROR;
}

uint32_t QCameraStream_Rdi::swapByteEndian(uint32_t jpegLength)
{
    jpegLength = (jpegLength >> 24) |
                 ((jpegLength << 8) & 0x00FF0000) |
                 ((jpegLength >> 8) & 0x0000FF00) |
                 (jpegLength << 24);
    return jpegLength;

}

status_t QCameraStream_Rdi::processJpegData(const void *jpegInfo, const void *jpegData,
                                            jpeg_info_t * jpeg_info, mm_camera_super_buf_t *jpeg_buf)
{
    status_t ret = NO_ERROR;
    uint8_t * NV12_meta = (uint8_t *)jpegInfo+0x800; //YUV meta offset 2048 bytes
    jpegInfo = (uint8_t *)jpegInfo+0xF; //First 16 bytes JPEG INFO start marker
    uint8_t jpegMode = *((uint8_t *)jpegInfo); // 1st byte 0x0
    uint8_t jpegCount = *((uint8_t *)jpegInfo + 1); //2nd byte 0x1
    uint32_t jpegLength = *((uint32_t *)jpegInfo + 1); // 2 reserved then 4 bytes 0x4
    uint32_t jpegFrameId = *((uint32_t *)jpegInfo + 2); // 4 bytes reserved 0x8
    uint32_t yuvFrameId = *((uint32_t *)(NV12_meta+0xE)); // 16 bytes of YUV meta start marker

    jpegLength = swapByteEndian(jpegLength);
    jpegFrameId = swapByteEndian(jpegFrameId);
    yuvFrameId = swapByteEndian(yuvFrameId);
    ALOGE("jpegMode %d jpegCount %x jpegLength %d, jpegFrameId %d yuvFrameId %d",
          jpegMode, jpegCount, (int)jpegLength, (int)jpegFrameId, (int)yuvFrameId);

    jpeg_info->NV12_meta = NV12_meta;
    jpeg_info->jpegInfo = (uint8_t *)jpegInfo;
    jpeg_info->jpegMode = jpegMode;
    jpeg_info->jpegCount = jpegCount;
    jpeg_info->jpegLength = jpegLength;
    jpeg_info->jpegFrameId = jpegFrameId;
    jpeg_info->yuvFrameId = yuvFrameId;

    if (jpegMode == META_MODE) {
        /* Meta  Data parsing */
    } else if (jpegMode == NORMAL_JPEG_MODE) {
        if(mJpegSuperBuf != NULL){
            free(mJpegSuperBuf);
            mJpegSuperBuf = NULL;
        }
        mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
        memcpy(mJpegSuperBuf, jpeg_buf, sizeof(mm_camera_super_buf_t));
        mJpegSuperBuf->split_jpeg = 0;
        *((uint32_t *)jpegInfo + 1) = jpegLength;
        ret = JPEG_RECEIVED;
    } else if (jpegMode == SPLIT_JPEG_MODE) {
        // Meta + splitted JPEG_1 or JPEG_2
        ALOGD("%s: split jpeg case: jpegCount %x jpegLength %d, yuvFrameId %d ",
            __func__, jpegCount, (int)jpegLength, (int)yuvFrameId);
        // if this is the first segment, alloc 16MB and copy, return JPEG_PENDING
        if (jpegCount == 0) {
           if(mJpegSuperBuf != NULL) {
               free(mJpegSuperBuf);
               mJpegSuperBuf = NULL;
           }
           mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
           mJpegSuperBuf->camera_handle = jpeg_buf->camera_handle;
           mJpegSuperBuf->ch_id = jpeg_buf->ch_id;
           mJpegSuperBuf->num_bufs = jpeg_buf->num_bufs;
           mJpegSuperBuf->bufs[0] = (mm_camera_buf_def_t*)malloc(sizeof(mm_camera_buf_def_t));
           mJpegSuperBuf->bufs[0]->buf_idx = jpeg_buf->bufs[0]->buf_idx;
           mJpegSuperBuf->bufs[0]->fd = jpeg_buf->bufs[0]->fd;
           mJpegSuperBuf->bufs[0]->frame_idx = jpeg_buf->bufs[0]->frame_idx;
           mJpegSuperBuf->bufs[0]->frame_len = jpeg_buf->bufs[0]->frame_len;
           mJpegSuperBuf->bufs[0]->mem_info = jpeg_buf->bufs[0]->mem_info;
           mJpegSuperBuf->bufs[0]->num_planes = jpeg_buf->bufs[0]->num_planes;
           memcpy(mJpegSuperBuf->bufs[0]->planes, jpeg_buf->bufs[0]->planes, sizeof(struct v4l2_plane *));
           mJpegSuperBuf->bufs[0]->stream_id = jpeg_buf->bufs[0]->stream_id;
           mJpegSuperBuf->bufs[0]->ts = jpeg_buf->bufs[0]->ts;
           mJpegSuperBuf->split_jpeg = 1;
           mJpegSuperBuf->bufs[0]->buffer = (void *)malloc(COMBINED_BUF_SIZE);
           mJpegSuperBuf->bufs[0]->frame_len = COMBINED_BUF_SIZE;
           memcpy(mJpegSuperBuf->bufs[0]->buffer, jpeg_buf->bufs[0]->buffer, JPEG_DATA_OFFSET);
           memcpy(mJpegSuperBuf->bufs[0]->buffer + JPEG_DATA_OFFSET, jpegData, jpegLength);

           //need to qbuf the old 8MB buffer
           //Since status is JPEG_PENDING, qbuf will be called 
           //at the end of processRdiFrame()
           ret = JPEG_PENDING;
        } else if (jpegCount == 1) {
           // if this is second segemnt, copy after first, return JPEG_RECEIVED
           memcpy(mJpegSuperBuf->bufs[0]->buffer + FIRST_JPEG_SIZE + JPEG_DATA_OFFSET, jpegData, jpegLength);
           *(uint32_t *)((uint8_t*)mJpegSuperBuf->bufs[0]->buffer+0x13) = jpegLength + FIRST_JPEG_SIZE;
           //need to qbuf the old 8MB buffer
           qbuf_helper(jpeg_buf);
           ret = JPEG_RECEIVED;
        }
     } else if (jpegMode >= HDR_JPEG_MODE && jpegMode <= BEST_PHOTO420_MODE) {
        ALOGD("%s: Received %d mode, Count %x ", __func__, jpegMode,  jpegCount);
        if(mJpegSuperBuf != NULL){
            free(mJpegSuperBuf);
            mJpegSuperBuf = NULL;
        }
        mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
        memcpy(mJpegSuperBuf, jpeg_buf, sizeof(mm_camera_super_buf_t));
        mJpegSuperBuf->split_jpeg = 0;
        *((uint32_t *)jpegInfo + 1) = jpegLength;
        ret = RAW_RECEIVED;
    } else {
        ALOGE("Error - %d mode This is not supported yet.", jpegMode);
        ret = BAD_VALUE;
    }
    ALOGD("%s: X", __func__);
    return ret;
}

status_t QCameraStream_Rdi::initRdiBuffers()
{
    status_t ret = NO_ERROR;
    int width = 0;  /* width of channel  */
    int height = 0; /* height of channel */
    const char *pmem_region;
    uint8_t num_planes = 0;
    mm_camera_frame_len_offset offset;
    uint32_t planes[VIDEO_MAX_PLANES];
    int i,  frame_len, y_off, cbcr_off;

    ALOGD("%s:BEGIN",__func__);

    width = mHalCamCtrl->mRdiWidth;
    height = mHalCamCtrl->mRdiHeight;

    mHalCamCtrl->mRdiMemoryLock.lock();
    memset(&mHalCamCtrl->mRdiMemory, 0, sizeof(mHalCamCtrl->mRdiMemory));
   mHalCamCtrl->mRdiMemory.buffer_count = mNumBuffers;//kRdiBufferCount;
    frame_len = mFrameOffsetInfo.frame_len;
    num_planes = mFrameOffsetInfo.num_planes;

    for ( i = 0; i < num_planes; i++) {
    planes[i] = mFrameOffsetInfo.mp[i].len;
    }
    y_off = 0;
    cbcr_off = planes[0];

    if (mHalCamCtrl->initHeapMem(&mHalCamCtrl->mRdiMemory,
     mHalCamCtrl->mRdiMemory.buffer_count,
     frame_len, y_off, cbcr_off, MSM_PMEM_MAINIMG,
     NULL,num_planes, planes) < 0) {
              ret = NO_MEMORY;
              return ret;
    };
    mHalCamCtrl->mRdiMemoryLock.unlock();

    for ( i = 0; i < num_planes; i++) {
    planes[i] = mFrameOffsetInfo.mp[i].len;
    }

    ALOGD("DEBUG: buffer count = %d",mHalCamCtrl->mRdiMemory.buffer_count);
    for (i = 0; i < mHalCamCtrl->mRdiMemory.buffer_count ; i++) {
      int j;

      mRdiBuf[i].fd = mHalCamCtrl->mRdiMemory.fd[i];
      mRdiBuf[i].frame_len = mHalCamCtrl->mRdiMemory.alloc[i].len;
      mRdiBuf[i].num_planes = num_planes;
      mRdiBuf[i].buffer= (void *)mHalCamCtrl->mRdiMemory.camera_memory[i]->data;

      /* Plane 0 needs to be set seperately. Set other planes
      * in a loop. */
      mFrameOffsetInfo.mp[0].len = mHalCamCtrl->mRdiMemory.alloc[i].len;

      mRdiBuf[i].planes[0].length = mFrameOffsetInfo.mp[0].len;
      mRdiBuf[i].planes[0].m.userptr = mRdiBuf[i].fd;
      mRdiBuf[i].planes[0].data_offset = mFrameOffsetInfo.mp[0].offset;
      mRdiBuf[i].planes[0].reserved[0] = 0;
      for (j = 1; j < num_planes; j++) {
          mRdiBuf[i].planes[j].length = mFrameOffsetInfo.mp[j].len;
          mRdiBuf[i].planes[j].m.userptr = mRdiBuf[i].fd;
          mRdiBuf[i].planes[j].data_offset = mFrameOffsetInfo.mp[j].offset;
          mRdiBuf[i].planes[j].reserved[0] =
              mRdiBuf[i].planes[j-1].reserved[0] +
              mRdiBuf[i].planes[j-1].length;
      }
      ALOGD(" %s : Buffer allocated %x fd = %d, length = %d num_planes = %d,"
            "mRdiBuf[i].planes[0].length = %d,mRdiBuf[i].planes[0].m.userptr = %d, mRdiBuf[i].planes[0].data_offset = %d",
             __func__, &mRdiBuf[i], mRdiBuf[i].fd,mRdiBuf[i].frame_len,num_planes,mRdiBuf[i].planes[0].length,
            mRdiBuf[i].planes[0].m.userptr,mRdiBuf[i].planes[0].data_offset);
    }

    end:
    if (MM_CAMERA_OK == ret ) {
      ALOGV("%s: X - NO_ERROR ", __func__);
      return NO_ERROR;
    }
    ALOGE("%s: X - BAD_VALUE ", __func__);
    return BAD_VALUE;
}

void QCameraStream_Rdi::dumpFrameToFile(mm_camera_buf_def_t* newFrame)
{
    char buf[32];
    int file_fd;
    int i;
    char *ext = "raw";
    int w, h;
    static int count = 0;
    char *name = "rdi";
    unsigned int size;
    w = mHalCamCtrl->mRdiWidth;
    h = mHalCamCtrl->mRdiHeight;

    if(LIKELY(count >= 10))
        return;

    if (newFrame != NULL) {
        char * str;
        size = w*h; /* buffer size without padding */
        snprintf(buf, sizeof(buf), "/data/%s_%d_%d_%d.%s", name, w, h, count,ext);
        file_fd = open(buf, O_RDWR | O_CREAT, 0777);
        if (file_fd < 0) {
            ALOGE("%s: cannot open file\n", __func__);
        } else {
            ALOGE("dumping RDI frame to file %s: size = %ld", buf, size);
            void* y_off = newFrame->buffer + newFrame->planes[0].data_offset;
            write(file_fd, (const void *)(y_off), size);
            close(file_fd);
        }
        count++;
    }
}

status_t QCameraStream_Rdi::processVisionModeFrame(
    mm_camera_super_buf_t *frame) {

    status_t rc = NO_ERROR;
    camera_data_callback pcb;
    camera_memory_t *data = NULL;
    camera_frame_metadata_t *metadata = NULL;
    uint32_t msgType=0x00;

    ALOGE("%s: E", __func__);
    /* Show preview FPS for debug*/
    if (UNLIKELY(mHalCamCtrl->mDebugFps)) {
        mHalCamCtrl->debugShowPreviewFPS();
    }

    Mutex::Autolock lock(mStopCallbackLock);

    mHalCamCtrl->mCallbackLock.lock();
    pcb = mHalCamCtrl->mDataCb;
    mHalCamCtrl->mCallbackLock.unlock();

    ALOGV("%s: MessageEnabled = 0x%x", __func__, mHalCamCtrl->mMsgEnabled);

    if(pcb != NULL) {
        if (mHalCamCtrl->mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) {
            msgType |=  CAMERA_MSG_PREVIEW_FRAME;
            data = mHalCamCtrl->mRdiMemory.camera_memory[frame->\
                bufs[0]->buf_idx];
        }
        if (msgType) {
            mStopCallbackLock.unlock();
            if (mActive) {
                ALOGV("sending data callback for vision mode");
                pcb(msgType, data, 0, metadata, mHalCamCtrl->mCallbackCookie);
            }
        }
    }
    return rc;
}

status_t QCameraStream_Rdi::processRdiFrame(
  mm_camera_super_buf_t *frame)
{
    ALOGV("%s: E",__func__);
    int err = 0;
    int msgType = 0;
    int i;
    status_t status = NO_ERROR, ret = NO_ERROR;
    camera_data_callback pcb;
    uint8_t * jpeg_info;

    if(mHalCamCtrl->mVisionModeFlag) {
        dumpFrameToFile(frame->bufs[0]);
        this->processVisionModeFrame(frame);
        qbuf_helper(frame);
        return NO_ERROR;
    }
    jpeg_info = (uint8_t *)(frame->bufs[0]->buffer);
    if (jpeg_info == NULL) {
        ALOGE("%s: Error: Received null jpeg_info", __func__);
        return BAD_VALUE;
    }
    Mutex::Autolock lock(mStopCallbackLock);
    if(!mActive) {
      ALOGE("RDI Streaming Stopped. Returning callback");
      ret = NO_ERROR;
      qbuf_helper(frame);
      return ret;
    }
    if(mHalCamCtrl==NULL) {
      ALOGE("%s: X: HAL control object not set",__func__);
      /*Call buf done*/
      ret = BAD_VALUE;
      qbuf_helper(frame);
      return ret;
    }
    mHalCamCtrl->mCallbackLock.lock();
    pcb = mHalCamCtrl->mDataCb;
    mHalCamCtrl->mCallbackLock.unlock();
    ALOGD("RDI2 frame idx %d", frame->bufs[0]->frame_idx);

    if (pcb != NULL) {
       //process JPEG info and data.
        jpeg_info_t jpegInfo;
        status = processJpegData(frame->bufs[0]->buffer,
                frame->bufs[0]->buffer + JPEG_DATA_OFFSET,&jpegInfo, frame);
        if(status == JPEG_PENDING){
            ALOGD("%s: split jpeg, finished with the first jpeg", __func__);
        }
        else if (status == JPEG_RECEIVED || status == RAW_RECEIVED) {
            /* set rawdata proc thread and jpeg notify thread to active state */
            mHalCamCtrl->mNotifyTh->sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, FALSE);
            mHalCamCtrl->mDataProcTh->sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, FALSE);
            mm_camera_super_buf_t* dup_frame =
                   (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
            if (dup_frame == NULL) {
                ALOGE("%s: Error allocating memory to save received_frame structure.", __func__);
                qbuf_helper(frame);
                ret = BAD_VALUE;
            } else {
                memcpy(dup_frame, mJpegSuperBuf, sizeof(mm_camera_super_buf_t));
                free (mJpegSuperBuf);
                mJpegSuperBuf = NULL;
                mHalCamCtrl->mSuperBufQueue.enqueue(dup_frame);
                if(status == RAW_RECEIVED){
                    ALOGD("For HDR Raw snapshot, just enqueue raw buffer and proceed ");
                    mHalCamCtrl->mNotifyTh->sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE);
                    return NO_ERROR;
                }
                ALOGV("Request_super_buf matching frame id %d ",jpegInfo.jpegFrameId);
                mm_camera_req_buf_t req_buf;
                req_buf.num_buf_requested = 1;
                req_buf.yuv_frame_id = jpegInfo.jpegFrameId;
                ret = mHalCamCtrl->mCameraHandle->ops->request_super_buf_by_frameId(
                        mHalCamCtrl->mCameraHandle->camera_handle,
                        mHalCamCtrl->mChannelId,
                        &req_buf);

                if (MM_CAMERA_OK != ret) {
                    ALOGE("error - request_super_buf failed.");
                    qbuf_helper(frame);
                }
                ret = NO_ERROR; //TODO: return or queue back the buff
            }
        } else if (ret != NO_ERROR) {
            ALOGE("processJpegData failed with %d", ret);
            qbuf_helper(frame);
            ret = BAD_VALUE;
        }  
    }
    if (status != JPEG_RECEIVED) {
        ALOGE("%s: Return RDI2 buffer back Qbuf\n", __func__);
        qbuf_helper(frame);
    }
    ALOGV("%s: X", __func__);
    return ret;
}

void QCameraStream_Rdi::qbuf_helper(mm_camera_super_buf_t *frame)
{
    for (int i=0; i<frame->num_bufs; i++) {
        if (frame->bufs[i] != NULL) {
            if(MM_CAMERA_OK != p_mm_ops->ops->qbuf(mCameraHandle, mChannelId,
                                    frame->bufs[i])){
                     ALOGE("%s: Buf done failed for buffer %p", __func__, frame->bufs[i]);
            }
        }
    }
}


// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------

QCameraStream_Rdi::
QCameraStream_Rdi(uint32_t CameraHandle,
                        uint32_t ChannelId,
                        uint32_t Width,
                        uint32_t Height,
                        uint32_t Format,
                        uint8_t NumBuffers,
                        mm_camera_vtbl_t *mm_ops,
                        mm_camera_img_mode imgmode,
                        camera_mode_t mode)
  : QCameraStream(CameraHandle,
                 ChannelId,
                 Width,
                 Height,
                 Format,
                 NumBuffers,
                 mm_ops,
                 imgmode,
                 mode),
    mNumFDRcvd(0)
{
    mHalCamCtrl = NULL;
    mJpegSuperBuf = NULL;
    ALOGV("%s: E", __func__);
    ALOGV("%s: X", __func__);
}
// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------

QCameraStream_Rdi::~QCameraStream_Rdi() {
    ALOGV("%s: E", __func__);
    if(mActive) {
        stop();
    }
    if(mInit) {
        release();
    }
    mInit = false;
    mActive = false;
    ALOGV("%s: X", __func__);

}
// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------

status_t QCameraStream_Rdi::init() {

    status_t ret = NO_ERROR;
    ALOGV("%s: E", __func__);
    return ret;
}
// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------

status_t QCameraStream_Rdi::start()
{
    ALOGV("%s: E", __func__);
    status_t ret = NO_ERROR;
    uint32_t stream_info;
    ALOGE("%s: X", __func__);
    return ret;
}


// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------
  void QCameraStream_Rdi::stop() {
    ALOGE("%s: E", __func__);
    int ret=MM_CAMERA_OK;
    uint32_t stream_info;
    uint32_t str[1];
    str[0] = mStreamId;

    ALOGE("%s : E", __func__);

    ret = p_mm_ops->ops->stop_streams(mCameraHandle, mChannelId, 1, str);
    if(ret != MM_CAMERA_OK){
      ALOGE("%s : stop_streams failed, ret = %d", __func__, ret);
    }
    ret= QCameraStream::deinitStream();
    ALOGE(": %s : De init Channel",__func__);
    if(ret != MM_CAMERA_OK) {
        ALOGE("%s:Deinit preview channel failed=%d\n", __func__, ret);
    }

    ALOGE("%s : X", __func__);
  }
// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------
  void QCameraStream_Rdi::release() {

    ALOGE("%s : BEGIN",__func__);
    int ret=MM_CAMERA_OK,i;

    if(!mInit)
    {
      ALOGE("%s : Stream not Initalized",__func__);
      return;
    }

    if(mActive) {
      this->stop();
    }
    ALOGE("%s: END", __func__);

  }

QCameraStream*
QCameraStream_Rdi::createInstance(uint32_t CameraHandle,
                        uint32_t ChannelId,
                        uint32_t Width,
                        uint32_t Height,
                        uint32_t Format,
                        uint8_t NumBuffers,
                        mm_camera_vtbl_t *mm_ops,
                        mm_camera_img_mode imgmode,
                        camera_mode_t mode)
{
  QCameraStream* pme = new QCameraStream_Rdi(CameraHandle,
                        ChannelId,
                        Width,
                        Height,
                        Format,
                        NumBuffers,
                        mm_ops,
                        imgmode,
                        mode);
  return pme;
}
// ---------------------------------------------------------------------------
// QCameraStream_Rdi
// ---------------------------------------------------------------------------

void QCameraStream_Rdi::deleteInstance(QCameraStream *p)
{
  if (p){
    ALOGV("%s: BEGIN", __func__);
    p->release();
    delete p;
    p = NULL;
    ALOGV("%s: END", __func__);
  }
}

// ---------------------------------------------------------------------------
// No code beyone this line
// ---------------------------------------------------------------------------
}; // namespace android
