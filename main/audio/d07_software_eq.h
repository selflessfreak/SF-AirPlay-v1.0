#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "settings.h"
#include "eq_events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

/* 15 peaking biquads, stereo, 48kHz, Q=1.0. Coefficients updated in
   playback task; web thread only writes desired dB under a spinlock. */
typedef struct { float b0,b1,b2,a1,a2,z1[2],z2[2]; } d07_eq_band_t;
static d07_eq_band_t d07_eq_filters[SETTINGS_EQ_BANDS];
static float d07_eq_requested[SETTINGS_EQ_BANDS];
static portMUX_TYPE d07_eq_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t d07_eq_version;
static uint32_t d07_eq_applied_version = UINT32_MAX;
static float d07_eq_headroom=1.0f;
static const float d07_eq_centers[SETTINGS_EQ_BANDS] = {
  20,31.5f,50,80,125,200,315,500,800,1250,2000,3150,5000,8000,16000
};
static void d07_eq_event_cb(eq_event_t event, const eq_event_data_t *data, void *user) {
  (void)user;
  if(event != EQ_EVENT_ALL_BANDS_SET || !data) return;
  float v[SETTINGS_EQ_BANDS];
  for(int k=0;k<SETTINGS_EQ_BANDS;k++){
    float g=data->all_bands.gains_db[k];
    v[k]=isfinite(g)?fmaxf(-15.0f,fminf(15.0f,g)):0.0f;
  }
  portENTER_CRITICAL(&d07_eq_lock);
  for(int k=0;k<SETTINGS_EQ_BANDS;k++) d07_eq_requested[k]=v[k];
  d07_eq_version++;
  portEXIT_CRITICAL(&d07_eq_lock);
  (void)settings_set_eq_gains(v);
}
static void d07_eq_init(void){
  float initial[SETTINGS_EQ_BANDS]={0};
  (void)settings_get_eq_gains(initial);
  portENTER_CRITICAL(&d07_eq_lock);
  for(int k=0;k<SETTINGS_EQ_BANDS;k++) d07_eq_requested[k]=initial[k];
  d07_eq_version++;
  portEXIT_CRITICAL(&d07_eq_lock);
  eq_events_register(d07_eq_event_cb,NULL);
}
static void d07_eq_refresh(void){
  float gains[SETTINGS_EQ_BANDS];
  uint32_t gen;
  portENTER_CRITICAL(&d07_eq_lock);
  gen=d07_eq_version;
  if(gen != d07_eq_applied_version)
    for(int k=0;k<SETTINGS_EQ_BANDS;k++) gains[k]=d07_eq_requested[k];
  portEXIT_CRITICAL(&d07_eq_lock);
  if(gen==d07_eq_applied_version) return;
  float largest=0.0f;
  for(int k=0;k<SETTINGS_EQ_BANDS;k++) if(gains[k]>largest) largest=gains[k];
  /* Conservative headroom. Filters still may accumulate boosts; final clamp protects PCM. */
  d07_eq_headroom=powf(10.0f,-fminf(15.0f,largest)/20.0f);
  for(int k=0;k<SETTINGS_EQ_BANDS;k++){
    float A=powf(10.0f,gains[k]/40.0f);
    float omega=6.28318530718f*d07_eq_centers[k]/48000.0f;
    float cs=cosf(omega), sn=sinf(omega), alpha=sn/2.0f;
    float norm=1.0f/(1.0f+alpha/A);
    d07_eq_band_t *b=&d07_eq_filters[k];
    b->b0=(1.0f+alpha*A)*norm;
    b->b1=(-2.0f*cs)*norm;
    b->b2=(1.0f-alpha*A)*norm;
    b->a1=(-2.0f*cs)*norm;
    b->a2=(1.0f-alpha/A)*norm;
    b->z1[0]=b->z1[1]=b->z2[0]=b->z2[1]=0.0f;
  }
  d07_eq_applied_version=gen;
}
static void d07_eq_process(int16_t *pcm,size_t frames){
  d07_eq_refresh();
  for(size_t i=0;i<frames;i++) for(int ch=0;ch<2;ch++){
    float x=(float)pcm[2*i+ch]*d07_eq_headroom;
    for(int k=0;k<SETTINGS_EQ_BANDS;k++){
      d07_eq_band_t *b=&d07_eq_filters[k];
      float y=b->b0*x+b->z1[ch];
      b->z1[ch]=b->b1*x-b->a1*y+b->z2[ch];
      b->z2[ch]=b->b2*x-b->a2*y;
      x=y;
    }
    if(x>32767.0f)x=32767.0f;
    if(x< -32768.0f)x=-32768.0f;
    pcm[2*i+ch]=(int16_t)x;
  }
}
