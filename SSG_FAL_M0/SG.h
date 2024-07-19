/*
 * ======================================================================================================================
 *  SG.h - Stream and Snow Distance Functions
 * ======================================================================================================================
 */

/*
 * Distance Sensors
 * The 5-meter sensors (MB7360, MB7369, MB7380, and MB7389) use a scale factor of (Vcc/5120) per 1-mm.
 * Particle 12bit resolution (0-4095),  Sensor has a resolution of 0 - 5119mm,  Each unit of the 0-4095 resolution is 1.25mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 5119mm, Each unit of the 0-1023 resolution is 5mm
 * 
 * The 10-meter sensors (MB7363, MB7366, MB7383, and MB7386) use a scale factor of (Vcc/10240) per 1-mm.
 * Particle 12bit resolution (0-4095), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-4095 resolution is 2.5mm
 * Feather has 10bit resolution (0-1023), Sensor has a resolution of 0 - 10239mm, Each unit of the 0-1023 resolution is 10mm
 */

#define SGAUGE_PIN     A3
#define SG_BUCKETS     60

unsigned int sg_bucket = 0;
unsigned int sg_buckets[SG_BUCKETS];

/* 
 *=======================================================================================================================
 * s_gauge_median()
 *=======================================================================================================================
 */
unsigned int s_gauge_median() {
  int i;

  for (i=0; i<SG_BUCKETS; i++) {
    // delay(500);
    delay(250);
    sg_buckets[i] = (int) analogRead(SGAUGE_PIN);
    // sprintf (Buffer32Bytes, "SG[%02d]:%d", i, sg_buckets[i]);
    // OutputNS (Buffer32Bytes);
  }
  
  mysort(sg_buckets, SG_BUCKETS);
  i = (SG_BUCKETS+1) / 2 - 1; // -1 as array indexing in C starts from 0

  if (cf_ds_type) {  // 0 = 5m, 1 = 10m
    return (sg_buckets[i]*5);
  }
  else {
    return (sg_buckets[i]*10);
  }
}
