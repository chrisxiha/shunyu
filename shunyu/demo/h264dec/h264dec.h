#ifndef __H264_DEC_H__
#define __H264_DEC_H__

typedef void* HDEC;

HDEC H264Dec_Init();
bool H264Dec_Decoder(HDEC hdec, void* h264_buffer, int h264_len, int *width, int *height, void* out_buffer, int * out_len);
void H264Dec_Uninit(HDEC hdec);

#endif

