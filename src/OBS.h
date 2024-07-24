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
void OBS_Do() {
  float rain1 = 0.0;
  float rain2 = 0.0;
  unsigned long rgds;     // rain gauge delta seconds, seconds since last rain gauge observation logged
  float BatteryPoC = 0.0; // Battery Percent of Charge
  float bmx1_pressure = 0.0;
  float bmx1_temp = 0.0;
  float bmx1_humid = 0.0;
  float bmx2_pressure = 0.0;
  float bmx2_temp = 0.0;
  float bmx2_humid = 0.0;
  float htu1_temp = 0.0;
  float htu1_humid = 0.0;
  float mcp1_temp = 0.0;
  float mcp2_temp = 0.0;
  float sht1_temp = 0.0;
  float sht1_humid = 0.0;
  float sht2_temp = 0.0;
  float sht2_humid = 0.0;
  float hih8_temp = 0.0;
  float hih8_humid = 0.0;
  float heat_index = 0.0;
  float wb_temp = 0.0;
  float wbg_temp = 0.0;

  // Safty Check for Vaild Time
  if (!Time.isValid()) {
    Output ("OBS_Do: Time NV");
    return;
  }

  int BatteryState = System.batteryState();
  // Read battery charge information only if battery is connected.
  if (BatteryState>0 && BatteryState<6) {
    BatteryPoC = System.batteryCharge();
  }

  // Get Battery Charger Failt Register
  byte cfr = pmic.getFault(); 

  float CellSignalStrength = 0;
  if (Cellular.ready()) {
    CellularSignal sig = Cellular.RSSI();
    CellSignalStrength = sig.getStrength();
  }

  // Rain Gauge 1 - Each tip is 0.2mm of rain
  rgds = (System.millis()-raingauge1_interrupt_stime)/1000;  // seconds since last rain gauge observation logged
  rain1 = raingauge1_interrupt_count * 0.2;
  raingauge1_interrupt_count = 0;
  raingauge1_interrupt_stime = System.millis();
  raingauge1_interrupt_ltime = 0; // used to debounce the tip
  // QC Check - Max Rain for period is (Observations Seconds / 60s) *  Max Rain for 60 Seconds
  rain1 = (isnan(rain1) || (rain1 < QC_MIN_RG) || (rain1 > ((rgds / 60) * QC_MAX_RG)) ) ? QC_ERR_RG : rain1;

  // Rain Gauge 2 - Each tip is 0.2mm of rain
  rgds = (System.millis()-raingauge2_interrupt_stime)/1000;  // seconds since last rain gauge observation logged
  rain2 = raingauge2_interrupt_count * 0.2;
  raingauge2_interrupt_count = 0;
  raingauge2_interrupt_stime = System.millis();
  raingauge2_interrupt_ltime = 0; // used to debounce the tip
  // QC Check - Max Rain for period is (Observations Seconds / 60s) *  Max Rain for 60 Seconds
  rain2 = (isnan(rain2) || (rain2 < QC_MIN_RG) || (rain2 > ((rgds / 60) * QC_MAX_RG)) ) ? QC_ERR_RG : rain2;

  EEPROM_UpdateRainTotals(rain1, rain2);

  // Adafruit I2C Sensors
  if (BMX_1_exists) {
    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      bmp1.takeForcedMeasurement(); // has no effect in normal mode
      bmx1_pressure = bmp1.readPressure()/100.0F;   // bp1 hPa
      bmx1_temp = bmp1.readTemperature();           // bt1
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) { 
      if (BMX_1_type == BMX_TYPE_BME280) {
        bme1.takeForcedMeasurement(); // has no effect in normal mode
        bmx1_pressure = bme1.readPressure()/100.0F;   // bp1 hPa
        bmx1_temp = bme1.readTemperature();           // bt1
        bmx1_humid = bme1.readHumidity();             // bh1 
      }
      if (BMX_1_type == BMX_TYPE_BMP390) {
        bmx1_pressure = bm31.readPressure()/100.0F;   // bp1 hPa
        bmx1_temp = bm31.readTemperature();           // bt1        
      }
    }
    else { // BMP388
      bmx1_pressure = bm31.readPressure()/100.0F;   // bp1 hPa
      bmx1_temp = bm31.readTemperature();           // bt1
    }

    // QC Checks
    bmx1_pressure = (isnan(bmx1_pressure) || (bmx1_pressure < QC_MIN_P)  || (bmx1_pressure > QC_MAX_P))  ? QC_ERR_P  : bmx1_pressure;
    bmx1_temp     = (isnan(bmx1_temp)     || (bmx1_temp     < QC_MIN_T)  || (bmx1_temp     > QC_MAX_T))  ? QC_ERR_T  : bmx1_temp;
    bmx1_humid    = (isnan(bmx1_humid)    || (bmx1_humid    < QC_MIN_RH) || (bmx1_humid    > QC_MAX_RH)) ? QC_ERR_RH : bmx1_humid;
  }

  if (BMX_2_exists) {
    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      bmp2.takeForcedMeasurement(); // has no effect in normal mode
      bmx2_pressure = bmp2.readPressure()/100.0F;   // bp2 hPa
      bmx2_temp = bmp2.readTemperature();           // bt2
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_2_type == BMX_TYPE_BME280) {
        bme2.takeForcedMeasurement(); // has no effect in normal mode
        bmx2_pressure = bme2.readPressure()/100.0F;   // bp2 hPa
        bmx2_temp = bme2.readTemperature();           // bt2
        bmx2_humid = bme2.readHumidity();             // bh2 
      }
      if (BMX_2_type == BMX_TYPE_BMP390) {
        bmx2_pressure = bm32.readPressure()/100.0F;   // bp2 hPa
        bmx2_temp = bm32.readTemperature();           // bt2       
      }
    }
    else { // BMP388
      bmx2_pressure = bm32.readPressure()/100.0F;   // bp2 hPa
      bmx2_temp = bm32.readTemperature();           // bt2
    }

    // QC Checks
    bmx2_pressure = (isnan(bmx2_pressure) || (bmx2_pressure < QC_MIN_P)  || (bmx2_pressure > QC_MAX_P))  ? QC_ERR_P  : bmx2_pressure;
    bmx2_temp     = (isnan(bmx2_temp)     || (bmx2_temp     < QC_MIN_T)  || (bmx2_temp     > QC_MAX_T))  ? QC_ERR_T  : bmx2_temp;
    bmx2_humid    = (isnan(bmx2_humid)    || (bmx2_humid    < QC_MIN_RH) || (bmx2_humid    > QC_MAX_RH)) ? QC_ERR_RH : bmx2_humid;
  }

  if (HTU21DF_exists) {
    htu1_temp = htu.readTemperature();
    htu1_humid= htu.readHumidity();
    // QC Checks
    htu1_temp =  (isnan(htu1_temp)  || (htu1_temp < QC_MIN_T)   || (htu1_temp > QC_MAX_T))   ? QC_ERR_T  : htu1_temp;
    htu1_humid = (isnan(htu1_humid) || (htu1_humid < QC_MIN_RH) || (htu1_humid > QC_MAX_RH)) ? QC_ERR_RH : htu1_humid;
  }

  if (MCP_1_exists) {
    float t = 0.0;
    t = mcp1.readTempC();
    // QC Checks
    mcp1_temp = (isnan(t) || (t < QC_MIN_T) || (t > QC_MAX_T)) ? QC_ERR_T : t;
  }

  if (MCP_2_exists) {
    float t = 0.0;
    t = mcp2.readTempC();
    // QC Checks
    mcp2_temp = (isnan(t) || (t < QC_MIN_T) || (t > QC_MAX_T)) ? QC_ERR_T : t;
  }

  if (SHT_1_exists) {
    float t = 0.0;
    float h = 0.0;
    t = sht1.readTemperature();
    h = sht1.readHumidity();
    // QC Checks
    sht1_temp  = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    sht1_humid = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (SHT_2_exists) {
    float t = 0.0;
    float h = 0.0;
    t = sht2.readTemperature();
    h = sht2.readHumidity();
    // QC Checks
    sht2_temp  = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    sht2_humid = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (HIH8_exists) {
    hih8_getTempHumid(&hih8_temp, &hih8_humid);
  }

  if (HI_exists) {
    heat_index = hi_calculate(mcp1_temp, sht1_humid);
  }

  if (WBT_exists) {
    wb_temp = wbt_calculate(mcp1_temp, sht1_humid);
  }

  if (WBGT_exists) {
    wbg_temp = wbgt_calculate(heat_index);
  }

  stc_timestamp();
  Output(timestamp);

  // Report if we have Need to Send Observations
  if (SD_exists && SD.exists(SD_n2s_file)) {
    SystemStatusBits |= SSB_N2S; // Turn on Bit
  }
  else {
    SystemStatusBits &= ~SSB_N2S; // Turn Off Bit
  }

  memset(msgbuf, 0, sizeof(msgbuf));
  JSONBufferWriter writer(msgbuf, sizeof(msgbuf)-1);
  writer.beginObject();
    writer.name("at").value(timestamp);
    writer.name("rg1").value(rain1,2);
    writer.name("rgt1").value(eeprom.rgt1,2);
    writer.name("rgp1").value(eeprom.rgp1,2);
    writer.name("rg2").value(rain2,2);
    writer.name("rgt2").value(eeprom.rgt2,2);
    writer.name("rgp2").value(eeprom.rgp2,2);

    if (BMX_1_exists) {
      writer.name("bp1").value(bmx1_pressure, 4);
      writer.name("bt1").value(bmx1_temp, 2);
      writer.name("bh1").value(bmx1_humid, 2);
    }
    if (BMX_2_exists) {
      writer.name("bp2").value(bmx2_pressure, 4);
      writer.name("bt2").value(bmx2_temp, 2);
      writer.name("bh2").value(bmx2_humid, 2);
    }

    if (HTU21DF_exists) {
      writer.name("ht1").value(htu1_temp, 2);
      writer.name("hh1").value(htu1_humid, 2);
    }

    if (MCP_1_exists) {
      writer.name("mt1").value(mcp1_temp, 2);
    }
    if (MCP_2_exists) {
      writer.name("mt2").value(mcp2_temp, 2);
    }

    if (SHT_1_exists) {
      writer.name("st1").value(sht1_temp, 2);
      writer.name("sh1").value(sht1_humid, 2);
    }
    if (SHT_2_exists) {
      writer.name("st2").value(sht2_temp, 2);
      writer.name("sh2").value(sht2_humid, 2);
    }

    if (HIH8_exists) {
      writer.name("ht2").value(hih8_temp, 2);
      writer.name("hh2").value(hih8_humid, 2);
    }

    if (HI_exists) {
      writer.name("hi").value(heat_index, 2);
    }

    if (WBT_exists) {
      writer.name("wbt").value(wb_temp, 2);
    }

    if (WBGT_exists) {
      writer.name("wbgt").value(wbg_temp, 2);
    }

    writer.name("bcs").value(BatteryState);
    writer.name("bpc").value(BatteryPoC, 2);
    writer.name("cfr").value(cfr);
    writer.name("css").value(CellSignalStrength, 2);
    writer.name("hth").value(SystemStatusBits);

    // International Mobile Subscriber Identity
    if (imsi_valid) {
      writer.name("imsi").value(imsi);
    }
    else {
      writer.name("imsi").value("NF");
    }
  writer.endObject();

  // Log Observation to SD Card
  SD_LogObservation(msgbuf);
  Serial_write (msgbuf);

  Time_of_last_obs = Time.now();

  Output ("Publish(FN)");
  if (Particle_Publish((char *) "FN")) { 
    PostedResults = true;

    if (SD_exists) {
      sprintf (Buffer32Bytes, "Publish(OK)[%d]", strlen(msgbuf)+1);
      Output (Buffer32Bytes);
    }
    else {
      Output ("Publish(OK)-NO SD!!!");
    }

    // If we Published, Lets try send N2S observations
    SD_N2S_Publish();
  }
  else {
    PostedResults = false;

    Output ("Publish(FAILED)");
    
    // Set the bit so when we finally transmit the observation,
    // we know it cam from the N2S file.
    SystemStatusBits |= SSB_FROM_N2S; // Turn On Bit

    memset(msgbuf, 0, sizeof(msgbuf));
    // SEE https://docs.particle.io/reference/device-os/firmware/argon/#jsonwriter
    JSONBufferWriter writer(msgbuf, sizeof(msgbuf)-1);
    writer.beginObject();
      writer.name("at").value(timestamp);
      writer.name("rg1").value(rain1,2);
      writer.name("rgt1").value(eeprom.rgt1,2);
      writer.name("rgp1").value(eeprom.rgp1,2);
      writer.name("rg2").value(rain2,2);
      writer.name("rgt2").value(eeprom.rgt2,2);
      writer.name("rgp2").value(eeprom.rgp2,2);

      if (BMX_1_exists) {
        // sprintf (Buffer32Bytes, "%d.%02d", (int)bmx1_pressure, (int)(bmx1_pressure*100)%100);
        // writer.name("bp1").value(Buffer32Bytes);
        writer.name("bp1").value(bmx1_pressure, 4);
        writer.name("bt1").value(bmx1_temp, 4);
        writer.name("bh1").value(bmx1_humid, 4);
      }
      if (BMX_2_exists) {
        // sprintf (Buffer32Bytes, "%d.%02d", (int)bmx2_pressure, (int)(bmx2_pressure*100)%100);
        // writer.name("bp2").value(Buffer32Bytes);
        writer.name("bp2").value(bmx2_pressure, 4);
        writer.name("bt2").value(bmx2_temp, 4);
        writer.name("bh2").value(bmx2_humid, 4);
      }
      if (HTU21DF_exists) {
        writer.name("ht1").value(htu1_temp, 2);
        writer.name("hh1").value(htu1_humid, 2);
      }

      if (MCP_1_exists) {
        writer.name("mt1").value(mcp1_temp, 2);
      }
      if (MCP_2_exists) {
        writer.name("mt2").value(mcp2_temp, 2);
      }

      if (SHT_1_exists) {
        writer.name("st1").value(sht1_temp, 2);
        writer.name("sh1").value(sht1_humid, 2);
      }
      if (SHT_2_exists) {
        writer.name("st2").value(sht2_temp, 2);
        writer.name("sh2").value(sht2_humid, 2);
      }

      if (HIH8_exists) {
        writer.name("ht2").value(hih8_temp, 2);
        writer.name("hh2").value(hih8_humid, 2);
      }

      writer.name("bcs").value(BatteryState);
      writer.name("bpc").value(BatteryPoC, 4);
      writer.name("cfr").value(cfr);
      writer.name("css").value(CellSignalStrength, 4);
      writer.name("hth").value(SystemStatusBits);

      // International Mobile Subscriber Identity
      if (imsi_valid) {
        writer.name("imsi").value(imsi);
      }
      else {
        writer.name("imsi").value("NF");
      }
    writer.endObject();
    SystemStatusBits &= ~SSB_FROM_N2S; // Turn Off Bit

    SD_NeedToSend_Add(msgbuf);
  }

  Output(timestamp);

  sprintf (msgbuf, "T%d.%01d T%d.%01d H%d.%01d",
    (int)bmx1_temp, (int)(bmx1_temp*10)%10, 
    (int)mcp1_temp, (int)(mcp1_temp*10)%10,
    (int)htu1_humid, (int)(htu1_humid*10)%10);
  Output(msgbuf);

  sprintf (msgbuf, "T%d.%01d H%d.%01d R%d.%01d R%d.%01d",
    (int)sht1_temp, (int)(sht1_temp*10)%10,
    (int)sht1_humid, (int)(sht1_humid*10)%10,
    (int)rain1, (int)(rain1*10)%10,
    (int)rain2, (int)(rain2*10)%10);
  Output(msgbuf);

  sprintf (msgbuf, "C%d.%02d %d:%d.%02d %04X", 
    (int)CellSignalStrength, (int)(CellSignalStrength*100)%100,
    BatteryState, 
    (int)BatteryPoC, (int)(BatteryPoC*100)%100,
    SystemStatusBits);
  Output(msgbuf);

  // Not outputed
  // (int)bmx2_temp, (int)(bmx2_temp*100)%100
  // (int)sht2_temp, (int)(sht2_temp*100)%100
  // (int)sht2_humid, (int)(sht2_humid*100)%100
}
