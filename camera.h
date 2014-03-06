#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <stdint.h>
#include <media/msm_camera.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <QCamera_Intf.h>

#define TRUE (1)
#define FALSE (0)
#define JPEG_EVENT_ERROR     2
#define JPEG_EVENT_ABORTED   3

#define JPEG_EVENT_DONE      0
#define JPEG_EVENT_THUMBNAIL_DROPPED 4

#define EXIFTAGID_GPS_LATITUDE_REF     0x10001
#define EXIFTAGID_GPS_LATITUDE         0x20002
#define EXIFTAGID_GPS_LONGITUDE_REF    0x30003
#define EXIFTAGID_GPS_LONGITUDE        0x40004
#define EXIFTAGID_GPS_ALTITUDE_REF     0x50005
#define EXIFTAGID_GPS_ALTITUDE         0x60006
#define EXIFTAGID_GPS_TIMESTAMP        0x70007
#define EXIFTAGID_GPS_PROCESSINGMETHOD 0x1b001b
#define EXIFTAGID_GPS_DATESTAMP        0x1d001d

#define EXIFTAGID_EXIF_DATE_TIME_ORIGINAL    0x939003
#define EXIFTAGID_EXIF_DATE_TIME_CREATED    0x949004
#define EXIFTAGID_FOCAL_LENGTH         0xa0920a
#define EXIFTAGID_ISO_SPEED_RATING     0x908827

#define CAMERA_MIN_BRIGHTNESS  0
#define CAMERA_DEF_BRIGHTNESS  3
#define CAMERA_MAX_BRIGHTNESS  6
#define CAMERA_BRIGHTNESS_STEP 1

#define CAMERA_MIN_CONTRAST    0
#define CAMERA_DEF_CONTRAST    5
#define CAMERA_MAX_CONTRAST    10

#define CAMERA_MIN_SCE_FACTOR    -100
#define CAMERA_DEF_SCE_FACTOR    0
#define CAMERA_MAX_SCE_FACTOR    100

/* No saturation for default */
#define CAMERA_MIN_SATURATION  0
#define CAMERA_DEF_SATURATION  5
#define CAMERA_MAX_SATURATION  10

/* No hue for default. */
#define CAMERA_MIN_HUE         0
#define CAMERA_DEF_HUE         0
#define CAMERA_MAX_HUE         300
#define CAMERA_HUE_STEP        60

/* No sharpness for default */
#define CAMERA_MIN_SHARPNESS   0
#define CAMERA_DEF_SHARPNESS   10
#define CAMERA_MAX_SHARPNESS   30
#define CAMERA_SHARPNESS_STEP  2

#define CAMERA_MIN_ZOOM  0
#define CAMERA_DEF_ZOOM  0
#define CAMERA_MAX_ZOOM  0x31
#define CAMERA_ZOOM_STEP 0x3

#define MAX_FRAGMENTS  8

#define JPEGERR_SUCCESS              0
#define JPEGERR_EFAILED              1

#define PAD_TO_2K(a)                 (((a)+2047)&~2047)

#define WAVELET_DENOISE_YCBCR_PLANE 0
#define WAVELET_DENOISE_CBCR_ONLY 1
#define WAVELET_DENOISE_STREAMLINE_YCBCR 2
#define WAVELET_DENOISE_STREAMLINED_CBCR 3

typedef enum
{
    YCRCBLP_H2V2 = 0,
    YCBCRLP_H2V2 = 1,

    YCRCBLP_H2V1 = 2,
    YCBCRLP_H2V1 = 3,

    YCRCBLP_H1V2 = 4,
    YCBCRLP_H1V2 = 5,

    YCRCBLP_H1V1 = 6,
    YCBCRLP_H1V1 = 7,

    RGB565 = 8,
    RGB888 = 9,
    RGBa   = 10,

    JPEG_BITSTREAM_H2V2 = 12,
    JPEG_BITSTREAM_H2V1 = 14,
    JPEG_BITSTREAM_H1V2 = 16,
    JPEG_BITSTREAM_H1V1 = 18,

    JPEG_COLOR_FORMAT_MAX,

} jpeg_color_format_t;


struct jpeg_buf_t;
typedef struct jpeg_buf_t * jpeg_buffer_t;

typedef uint32_t  jpeg_event_t;

struct jpeg_encoder_t;
typedef struct jpeg_encoder_t *jpege_obj_t;

struct exif_info_t;
typedef struct exif_info_t * exif_info_obj_t;

typedef struct {
    union {
        struct {
            jpeg_buffer_t luma_buf;
            jpeg_buffer_t chroma_buf;
        } yuv;
        struct {
            jpeg_buffer_t rgb_buf;
        } rgb;
        struct {
            jpeg_buffer_t bitstream_buf;
        } bitstream;
    } color;
    uint32_t   width;
    uint32_t   height;

} jpege_img_frag_t;
typedef struct
{
    jpeg_color_format_t color_format;
    uint32_t            width;
    uint32_t            height;
    uint32_t            fragment_cnt;
    jpege_img_frag_t    p_fragments[MAX_FRAGMENTS];

} jpege_img_data_t;
typedef struct
{
    jpege_img_data_t  *p_main;
    jpege_img_data_t  *p_thumbnail;
} jpege_src_t;

typedef  int(*jpege_output_handler_t)(void *p_user_data,void *p_arg,jpeg_buffer_t buf,uint8_t last_buf_flag);
typedef struct
{
    jpege_output_handler_t  p_output_handler;

    void                   *p_arg;

    uint32_t                buffer_cnt;
    jpeg_buffer_t           *p_buffer;
    jpeg_buffer_t           buffers[2];

} jpege_dst_t;

typedef struct
{
    uint32_t    input_width;
    uint32_t    input_height;
    uint32_t    h_offset;
    uint32_t    v_offset;
    uint32_t    output_width;
    uint32_t    output_height;
    uint8_t     enable;

} jpege_scale_cfg_t;

typedef struct
{
    uint8_t bits[17];
    uint8_t values[256];

} jpeg_huff_table_t;

typedef uint16_t* jpeg_quant_table_t;

typedef struct
{
    uint32_t              quality;
    uint32_t              rotation_degree_clk;
    jpeg_quant_table_t    luma_quant_tbl;
    jpeg_quant_table_t    chroma_quant_tbl;
    jpeg_huff_table_t     luma_dc_huff_tbl;
    jpeg_huff_table_t     chroma_dc_huff_tbl;
    jpeg_huff_table_t     luma_ac_huff_tbl;
    jpeg_huff_table_t     chroma_ac_huff_tbl;
    uint32_t              restart_interval;
    jpege_scale_cfg_t     scale_cfg;

} jpege_img_cfg_t;

typedef enum
{
    OUTPUT_EXIF = 0,
    OUTPUT_JFIF

} jpege_hdr_output_t;
typedef enum
{
    JPEG_ENCODER_PREF_HW_ACCELERATED_PREFERRED = 0,
    JPEG_ENCODER_PREF_HW_ACCELERATED_ONLY,
    JPEG_ENCODER_PREF_SOFTWARE_PREFERRED,
    JPEG_ENCODER_PREF_SOFTWARE_ONLY,
    JPEG_ENCODER_PREF_DONT_CARE,

    JPEG_ENCODER_PREF_MAX,

} jpege_preference_t;


typedef struct
{
    uint32_t             target_filesize;
    jpege_hdr_output_t   header_type;
    jpege_img_cfg_t      main_cfg;
    jpege_img_cfg_t      thumbnail_cfg;
    uint8_t              thumbnail_present;
    jpege_preference_t   preference;
    uint32_t            app2_header_length;

} jpege_cfg_t;

typedef struct video_dis_param_ctrl_t {
  uint32_t dis_enable;       /* DIS feature: 1 = enable, 0 = disable.
                               when enable, caller makes sure w/h are 10% more. */
  uint32_t video_rec_width;  /* video frame width for recording */
  uint32_t video_rec_height; /* video frame height for recording */
  uint32_t output_cbcr_offset;
} video_dis_param_ctrl_t;

typedef enum VIDEO_ROT_ENUM {
  ROT_NONE               = 0,
  ROT_CLOCKWISE_90       = 1,
  ROT_CLOCKWISE_180      = 6,
  ROT_CLOCKWISE_270      = 7,
} VIDEO_ROT_ENUM;


typedef struct video_rotation_param_ctrl_t {
  VIDEO_ROT_ENUM rotation; /* 0 degree = rot disable. */
} video_rotation_param_ctrl_t;

enum focus_distance_index{
  FOCUS_DISTANCE_NEAR_INDEX,  /* 0 */
  FOCUS_DISTANCE_OPTIMAL_INDEX,
  FOCUS_DISTANCE_FAR_INDEX,
  FOCUS_DISTANCE_MAX_INDEX
};

typedef struct video_frame_info {

  uint32_t               y_buffer_width;     /* y plane */
  uint32_t               cbcr_buffer_width;  /* cbcr plane */
  uint32_t               image_width;        /**< original image width.   */
  uint32_t               image_height;       /**< original image height. */
  uint32_t               color_format;
} video_frame_info;

typedef struct {
  float focus_distance[FOCUS_DISTANCE_MAX_INDEX];
} focus_distances_info_t;

typedef enum {
  _SIDE_BY_SIDE_HALF,
  _SIDE_BY_SIDE_FULL,
  _TOP_DOWN_HALF,
  _TOP_DOWN_FULL,
}cam_3d_frame_format_t;

typedef enum {
  AUTO = 1,
  SPOT,
  CENTER_WEIGHTED,
  AVERAGE
} cam_af_focusrect_t;

typedef struct {
  int32_t  buffer[256];
  int32_t  max_value;
} camera_preview_histogram_info;


typedef struct {
  cam_frame_type_t frame_type;
  cam_3d_frame_format_t format;
}camera_3d_frame_t;

typedef enum {
  FPS_MODE_AUTO,
  FPS_MODE_FIXED,
} fps_mode_t;

#endif /* __CAMERA_H__ */
