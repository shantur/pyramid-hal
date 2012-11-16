/*
** Copyright (c) 2011-2012 The Linux Foundation. All rights reserved.
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
#include <genlock.h>

#define UNLIKELY(exp) __builtin_expect(!!(exp), 0)

/* QCameraHWI_Raw class implementation goes here*/
/* following code implement the RDI logic of this class*/

namespace android {
#define JPEG_RECEIVED 1
#define JPEG_PENDING 2
#define RAW_RECEIVED 3

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
#if 1 // QCT 10162012 RDI2 changes 
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
    ALOGE("Before swap: %x", jpegLength);
    jpegLength = (jpegLength >> 24) |
                 ((jpegLength << 8) & 0x00FF0000) |
                 ((jpegLength >> 8) & 0x0000FF00) |
                 (jpegLength << 24);
    ALOGE("After swap: %x", jpegLength);
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
    ALOGE("yuvFrameId %x (NV12_meta+0xE) %x %x %x %x",yuvFrameId,  
          *(NV12_meta+0xE), *(NV12_meta+0xF), *(NV12_meta+0x10), *(NV12_meta+0x11));
    yuvFrameId = swapByteEndian(yuvFrameId);
    ALOGE("jpegMode %d jpegCount %d jpegLength %d, jpegFrameId %d yuvFrameId %d", 
          jpegMode, jpegCount, (int)jpegLength, (int)jpegFrameId, (int)yuvFrameId);

    jpeg_info->NV12_meta = NV12_meta;
    jpeg_info->jpegInfo = (uint8_t *)jpegInfo;
    jpeg_info->jpegMode = jpegMode;
    jpeg_info->jpegCount = jpegCount;
    jpeg_info->jpegLength = jpegLength;
    jpeg_info->jpegFrameId = jpegFrameId;
    jpeg_info->yuvFrameId = yuvFrameId;

    if (jpegMode == 0x00) {
        // Meta
    } else if (jpegMode == 0x01) {
        if(mJpegSuperBuf != NULL){
            ALOGD("%s: mJpegSuperBuf was not null, so free and null", __func__);
            free(mJpegSuperBuf);
            mJpegSuperBuf = NULL;
        }
        mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
        ALOGD("%s: superbuf address = new = %x, old =%x", __func__, mJpegSuperBuf, jpeg_buf);
        memcpy(mJpegSuperBuf, jpeg_buf, sizeof(mm_camera_super_buf_t));
        mJpegSuperBuf->split_jpeg = 0;
        *((uint32_t *)jpegInfo + 1) = jpegLength;
        ALOGD("%s jpegLength = %d, jpegLength written to superbuf = %d, jpeg_length_ptr = %x", 
              __func__, jpegLength, *((uint32_t *)jpegInfo + 1), (uint32_t *)jpegInfo + 1);
        ret = JPEG_RECEIVED;
    }
#ifdef RDI2_SPLIT
     else if (jpegMode == 0x02) {
        // Meta + splitted JPEG_1 or JPEG_2

        ALOGD("%s: split jpeg case: jpegCount %d jpegLength %d, yuvFrameId %d ",
            __func__, jpegCount, (int)jpegLength, (int)yuvFrameId);
        // if this is the first segment, alloc 16MB and copy, return JPEG_PENDING
        if (jpegCount == 0)
        {
           ALOGD("%s: first jpeg received", __func__);
           if(mJpegSuperBuf != NULL){
               ALOGD("%s: mJpegSuperBuf was not null, so free and null", __func__);
               free(mJpegSuperBuf);
               ALOGD("%s: mJpegSuperBuf was not null, after free", __func__);
               mJpegSuperBuf = NULL;
           }
           ALOGD("%s: allocating memory for the mJpegSuperBuf", __func__);
           mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
           memcpy (mJpegSuperBuf, jpeg_buf, sizeof(mm_camera_super_buf_t));
           mJpegSuperBuf->split_jpeg = 1;
           mJpegSuperBuf->bufs[0]->buffer = (void *)malloc(0x1000000);
           mJpegSuperBuf->bufs[0]->frame_len = 0x1000000;
           // make a 16MB for jpeg_buf_dup, and copy data
           ALOGD("%s:mJpegSuperBuf is at %p, jpeg_buf is at %p", __func__, mJpegSuperBuf, jpeg_buf);
           ALOGD("%s:mJpegSuperBuf->bufs[0]->buffer is at %p", __func__, mJpegSuperBuf->bufs[0]->buffer);

           memcpy(mJpegSuperBuf->bufs[0]->buffer, jpeg_buf->bufs[0]->buffer, JPEG_DATA_OFFSET);
           memcpy(mJpegSuperBuf->bufs[0]->buffer+JPEG_DATA_OFFSET, jpegData, jpegLength);

           //need to qbuf the old 8MB buffer
           for (int i=0; i<jpeg_buf->num_bufs; i++) {
             if (jpeg_buf->bufs[i] != NULL) {
                ALOGD("%s: qbuf the old 8MB buf first jpeg", __func__);
                if(MM_CAMERA_OK != p_mm_ops->ops->qbuf(mCameraHandle, mChannelId,
                                    jpeg_buf->bufs[i])){
                     ALOGE("%s: Buf done failed for buffer %p", __func__, jpeg_buf->bufs[i]);
                 }
             }
           }
           ret = JPEG_PENDING;

        } else if (jpegCount == 1) {
          ALOGD("%s: second jpeg received", __func__);
        // if this is second segemnt, copy after first, return JPEG_RECEIVED
           memcpy(mJpegSuperBuf->bufs[0]->buffer+ 0x802800, jpegData, jpegLength);

           ALOGD("%s: address writing to is %p", __func__, (uint32_t *)((uint8_t*)mJpegSuperBuf->bufs[0]->buffer+0x13));
           *(uint32_t *)((uint8_t*)mJpegSuperBuf->bufs[0]->buffer+0x13) = jpegLength + 0x800000;
           ALOGD("%s: count = 1, mJpegSuperBuf->bufs[0]->buffer is at %p", __func__, mJpegSuperBuf->bufs[0]->buffer);
           //need to qbuf the old 8MB buffer
           for (int i=0; i<jpeg_buf->num_bufs; i++) {
             if (jpeg_buf->bufs[i] != NULL) {
                ALOGE("%s: qbuf the old 8MB buf second jpeg", __func__);
                if(MM_CAMERA_OK != p_mm_ops->ops->qbuf(mCameraHandle, mChannelId,
                                    jpeg_buf->bufs[i])){
                         ALOGE("%s: Buf done failed for buffer %p", __func__, jpeg_buf->bufs[i]);
                    }
                }
             }
           ret = JPEG_RECEIVED;
        }
     }
#endif
    else if (jpegMode == 0x10) {
        ALOGD("%d : Received HDR Raw data. Count %d ",jpegMode,  jpegCount);
        if(mJpegSuperBuf != NULL){
            free(mJpegSuperBuf);
            mJpegSuperBuf = NULL;
        }
        ALOGD("%s: allocating memory for the mJpegSuperBuf", __func__);
        mJpegSuperBuf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
        ALOGD("%s: superbuf address = new = %x, old =%x", __func__, mJpegSuperBuf, jpeg_buf);
        memcpy(mJpegSuperBuf, jpeg_buf, sizeof(mm_camera_super_buf_t));
        mJpegSuperBuf->split_jpeg = 0;
        *((uint32_t *)jpegInfo + 1) = jpegLength;
        ALOGD("%s jpegLength = %d, jpegLength written to superbuf = %d, jpeg_length_ptr = %x",
              __func__, jpegLength, *((uint32_t *)jpegInfo + 1), (uint32_t *)jpegInfo + 1);
        ALOGD("%s: mode = 1, mJpegSuperBuf is at %p", __func__, mJpegSuperBuf);
        ret = RAW_RECEIVED;
    }
    else {
        // Meta + splitted JPEG_1 or JPEG_2
        ALOGD("jpegCount %d jpegLength %d, yuvFrameId %d ", jpegCount, 
                                              (int)jpegLength, (int)yuvFrameId);
        ALOGD("Error - %d mode This is not supported yet.", jpegMode);
        ret = BAD_VALUE;
    }
    ALOGD("%s: X", __func__);
    return ret;
}
#endif
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
    char *ext = "yuv";
    int w,h,main_422;
    static int count = 0;
    char *name = "rdi";

    w = mHalCamCtrl->mRdiWidth;
    h = mHalCamCtrl->mRdiHeight;
#if 1 // QCT 10162012 RDI2 changes 
    w = 2048;
    h = 4101;
#endif
    main_422 = 1;

    if ( newFrame != NULL) {
        char * str;
        snprintf(buf, sizeof(buf), "/data/%s_%d.%s", name,count,ext);
        file_fd = open(buf, O_RDWR | O_CREAT, 0777);
        if (file_fd < 0) {
            ALOGE("%s: cannot open file\n", __func__);
        } else {
            void* y_off = newFrame->buffer + newFrame->planes[0].data_offset;
            void* cbcr_off = newFrame->buffer + newFrame->planes[0].length;

            write(file_fd, (const void *)(y_off), newFrame->planes[0].length);
#if 0 // QCT 10162012 RDI2 changes  // original 0
            write(file_fd, (const void *)(cbcr_off), 
                  (newFrame->planes[1].length * newFrame->num_planes));
#endif
            close(file_fd);
        }
        count++;
    }
}

status_t QCameraStream_Rdi::processRdiFrame(
  mm_camera_super_buf_t *frame)
{
    ALOGV("%s",__func__);
    int err = 0;
    int msgType = 0;
    int i;
#if 1 // QCT 10162012 RDI2 changes
    status_t status = NO_ERROR;
    camera_data_callback pcb;
#endif
    camera_memory_t *data = NULL;
    uint8_t * jpeg_info;
    uint8_t * jpeg_data;

    Mutex::Autolock lock(mStopCallbackLock);
    if(!mActive) {
      ALOGE("RDI Streaming Stopped. Returning callback");
      return NO_ERROR;
    }
    if(mHalCamCtrl==NULL) {
      ALOGE("%s: X: HAL control object not set",__func__);
    /*Call buf done*/
      status = BAD_VALUE;
      goto done;
    }
    mHalCamCtrl->mCallbackLock.lock();
    pcb = mHalCamCtrl->mDataCb;
    mHalCamCtrl->mCallbackLock.unlock();

    jpeg_info = (uint8_t *)(frame->bufs[0]->buffer);
    ALOGD("jpeg_info %x", jpeg_info);
    if(NULL != jpeg_info) {
        jpeg_data = jpeg_info + 0x2800;
        ALOGE("jpegMode %x \n", *((uint8_t *)(jpeg_info+0xF)));
        ALOGD("RDI2 frame idx %d", frame->bufs[0]->frame_idx);

        if(*(uint8_t *)(jpeg_info + 0xF)) {
            ALOGE("JPEG RECEIVED!!!!");
#if 0 //debugging
            ALOGE("dump the frame");
            dumpFrameToFile(frame->bufs[0]);
#endif
        }
    } else {
        ALOGE("got an NULL buffer on frame %p", frame);
        }
    //dumpFrameToFile(frame->bufs[0]);
#if 0 // QCT 10162012 RDI2 changes  //original 0
    if (pcb != NULL) {
      //Sending rdi callback if corresponding Msgs are enabled
      if(mHalCamCtrl->mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) {
          msgType |=  CAMERA_MSG_PREVIEW_FRAME;
        data = mHalCamCtrl->mRdiMemory.camera_memory[frame->bufs[0]->buf_idx];
      } else {
          data = NULL;
      }

      if(msgType) {
          mStopCallbackLock.unlock();
          if(mActive)
            pcb(msgType, data, 0, NULL, mHalCamCtrl->mCallbackCookie);
      }
      ALOGD("end of cb");
    }
#endif
if (pcb != NULL) {

        //process YUV metadata
     /*   status = updatePreviewMetadata(frame->bufs[0]->buffer + YUV_META_OFFSET);
        if (status != NO_ERROR) {
            ALOGE("updatePreviewMetadata failed with %d", status);
            goto done;
        }*/

        //process JPEG info and data.
        jpeg_info_t jpegInfo;
        status = processJpegData(frame->bufs[0]->buffer,
                frame->bufs[0]->buffer + JPEG_DATA_OFFSET,&jpegInfo, frame);
        if(status == JPEG_PENDING){
            ALOGD("%s: split jpeg, finished with the first jpeg", __func__);
        }
        else if (status == JPEG_RECEIVED || status == RAW_RECEIVED) {
            ALOGD("%s: frame addr %x num_bufs %d", __func__, frame, frame->num_bufs );
            mm_camera_super_buf_t* dup_frame =
                   (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
            if (dup_frame == NULL) {
                ALOGE("%s: Error allocating memory to save received_frame structure.", __func__);
                return BAD_VALUE;
                goto done;
            }
#ifdef RDI2_SPLIT
            memcpy(dup_frame, mJpegSuperBuf, sizeof(mm_camera_super_buf_t));
            free (mJpegSuperBuf);
            mJpegSuperBuf = NULL;
#else
            memcpy(dup_frame, frame, sizeof(mm_camera_super_buf_t));
#endif
            mHalCamCtrl->mSuperBufQueue.enqueue(dup_frame);
        
#if RDI2_THUMB //thumbnail
#if RDI2_YUV_SYNC
            ALOGE("Request_super_buf matching frame id %d ",jpegInfo.jpegFrameId);
            mm_camera_req_buf_t req_buf;
            req_buf.num_buf_requested = 1;
            req_buf.yuv_frame_id = jpegInfo.jpegFrameId;
            status = mHalCamCtrl->mCameraHandle->ops->request_super_buf_by_frameId(
                    mHalCamCtrl->mCameraHandle->camera_handle,
                    mHalCamCtrl->mChannelId,
                    &req_buf);
#else
            ALOGD("Request_super_buf .");
            status = mHalCamCtrl->mCameraHandle->ops->request_super_buf(
                    mHalCamCtrl->mCameraHandle->camera_handle,
                    mHalCamCtrl->mChannelId,
                    1)
#endif
            if (MM_CAMERA_OK != status) {
                    ALOGE("error - request_super_buf failed.");
                    goto done;
            }
#else // if thumbnail changes not taken, issue DO_NEXT_JOB from here.
           mHalCamCtrl->mNotifyTh->sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE);
#endif

            return NO_ERROR; //TODO: return or queue back the buff
        }
        if (status != NO_ERROR) {
            ALOGE("processJpegData failed with %d", status);
            goto done;
        }
    }

done:
    if (status != JPEG_RECEIVED) {
        ALOGE("%s: Return RDI2 buffer back Qbuf\n", __func__);
        for (int i=0; i<frame->num_bufs; i++) {
             if (frame->bufs[i] != NULL) {
                p_mm_ops->ops->qbuf(mCameraHandle, mChannelId,
                                                frame->bufs[i]);
                }
        }
    }
    return status;
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
