/*
 * ======================================================================================================================
 *  OBS.h - Observation Handeling
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * OBS_Do() - Collect Observations, Build message, Send to logging site
 * ======================================================================================================================
 */
void OBS_Do (bool log_obs) {
  float bmx1_pressure = 0.0;
  float bmx1_temp = 0.0;
  float bmx1_humid = 0.0;
  float bmx2_pressure = 0.0;
  float bmx2_temp = 0.0;
  float bmx2_humid = 0.0;
  float mcp1_temp = 0.0;
  float batt = 0.0;
  int msgLength;
  unsigned short checksum;

  // Safty Check for Vaild Time
  if (!RTC_valid) {
    Output ("OBS_Do: Time NV");
    return;
  }

  Output ("OBS_Do()");
 
  // Take multiple readings and return the median, 15s spent reading guage
  int SG_Median = s_gauge_median();
  
  //
  // Add I2C Sensors
  //
  if (BMX_1_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      p = bmp1.readPressure()/100.0F;       // bp1 hPa
      t = bmp1.readTemperature();           // bt1
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_1_type == BMX_TYPE_BME280) {
        p = bme1.readPressure()/100.0F;     // bp1 hPa
        t = bme1.readTemperature();         // bt1
        h = bme1.readHumidity();            // bh1
      }
      if (BMX_1_type == BMX_TYPE_BMP390) {
        p = bm31.readPressure()/100.0F;     // bp1 hPa
        t = bm31.readTemperature();         // bt1       
      }
    }
    else { // BMP388
      p = bm31.readPressure()/100.0F;       // bp1 hPa
      t = bm31.readTemperature();           // bt1
    }
    bmx1_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx1_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx1_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (BMX_2_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      p = bmp2.readPressure()/100.0F;       // bp2 hPa
      t = bmp2.readTemperature();           // bt2
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_2_type == BMX_TYPE_BME280) {
        p = bme2.readPressure()/100.0F;     // bp2 hPa
        t = bme2.readTemperature();         // bt2
        h = bme2.readHumidity();            // bh2 
      }
      if (BMX_2_type == BMX_TYPE_BMP390) {
        p = bm32.readPressure()/100.0F;       // bp2 hPa
        t = bm32.readTemperature();           // bt2
      }      
    }
    else { // BMP388
      p = bm32.readPressure()/100.0F;       // bp2 hPa
      t = bm32.readTemperature();           // bt2
    }
    bmx2_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx2_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx2_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (MCP_1_exists) {
    mcp1_temp = mcp1.readTempC();
    mcp1_temp = (isnan(mcp1_temp) || (mcp1_temp < QC_MIN_T)  || (mcp1_temp > QC_MAX_T))  ? QC_ERR_T  : mcp1_temp;
  }
  
  batt = vbat_get();

  // Set the time for this observation
  rtc_timestamp();
  if (log_obs) {
    Output(timestamp);
  }
  
  // Build JSON log entry by hand  
  // {"at":"2021-03-05T11:43:59","sg":49,"bp1":3,"bt1":97.875,"bh1":40.20,"bv":3.5,"hth":9}

  sprintf (msgbuf, "{\"at\":\"%s\",\"sg\":%d,", timestamp, SG_Median);
  if (BMX_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp1\":%u.%04d,\"bt1\":%d.%02d,\"bh1\":%d.%02d,",
      (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
      (int)bmx1_temp, (int)(bmx1_temp*100)%100,
      (int)bmx1_humid, (int)(bmx1_humid*100)%100);
  }
  if (BMX_2_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp2\":%u.%04d,\"bt2\":%d.%02d,\"bh2\":%d.%02d,",
      (int)bmx2_pressure, (int)(bmx2_pressure*100)%100,
      (int)bmx2_temp, (int)(bmx2_temp*100)%100,
      (int)bmx2_humid, (int)(bmx2_humid*100)%100);
  }
  if (MCP_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"mt1\":%d.%04d,", (int)mcp1_temp, (int)(mcp1_temp*100)%100);   
  }
  if (ds_found) {
    getDSTemp();
    sprintf (msgbuf+strlen(msgbuf), "\"dt1\":%d.%04d,", (int)ds_reading, (int)(ds_reading*100)%100);   
  }
  sprintf (msgbuf+strlen(msgbuf), "\"bv\":%d.%02d,\"hth\":%d}", 
    (int)batt, (int)(batt*100)%100, SystemStatusBits);

  // Log Observation to SD Card
  if (log_obs) {
    SD_LogObservation(msgbuf);
  }
  Serial_write (msgbuf);
}
