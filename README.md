# RF210_SB


  "VER", "Return firmware version", "VER", ATC_Ver, RAK_ATCMD_PERM_READ);

  "SCDCO2", "Return the status of the SHTC3 sensor. 1 if available.", "SCDCO2", scd41_co2, RAK_ATCMD_PERM_READ);
  "SCDTEMP", "Return the temperature value with 0.01° resolution", "SCDTEMP", SHTC3_temp, RAK_ATCMD_PERM_READ);
  "SCDHUM", "Return the humidity value with 1% resolution", "SCDHUM", SHTC3_humi, RAK_ATCMD_PERM_READ);

  "BMEGAS", "Return the gaz resistivity in ohmable.", "BMEGAS", bme_gas, RAK_ATCMD_PERM_READ);
  "BMETEMP", "Return the temperature value with 0.01° resolution", "BMETEMP", bme_temp, RAK_ATCMD_PERM_READ);
  "BMEHUM", "Return the humidity value with 1% resolution", "BMEHUM", bme_hum, RAK_ATCMD_PERM_READ);
  "BMEBAR", "Return the pressure value in mbar ", "BMEHUM", bme_bar, RAK_ATCMD_PERM_READ);

  "TEMP", "Return the temperature value with 0.01° resolution", "TEMP", SHTC3_temp, RAK_ATCMD_PERM_READ);
  "HUM", "Return the humidity value with 1% resolution", "HUM", SHTC3_humi, RAK_ATCMD_PERM_READ);

  "KX023", "Return the status of the KX023 sensor. 1 if available.", "KX023", KX023_init, RAK_ATCMD_PERM_READ);
  "AX", "Return the value of X acceleration with 0.01G resolution", "AX", KX023_AX, RAK_ATCMD_PERM_READ);
  "AY", "Return the value of Y acceleration with 0.01G resolution", "AY", KX023_AY, RAK_ATCMD_PERM_READ);
  "AZ", "Return the value of Z acceleration with 0.01G resolution", "AZ", KX023_AZ, RAK_ATCMD_PERM_READ);

  "LTR", "Return the status of the LTR-303 sensor. 1 if available.", "LTR", LTR_init, RAK_ATCMD_PERM_READ);
  "LUMCH0", "Return the CHANNEL0 value of the LTR-303 sensor", "LUMCH0", LTR_ch0, RAK_ATCMD_PERM_READ);
  "LUMCH1", "Return the CHANNEL1 value of the LTR-303 sensor", "LUMCH1", LTR_ch1, RAK_ATCMD_PERM_READ);
  "LUM", "Return the CHANNEL1 value of the LTR-303 sensor", "LUM", LTR_lux, RAK_ATCMD_PERM_READ);


  "BAT", "Return battery voltage in mV | Return 0 if not available", "BAT", battery, RAK_ATCMD_PERM_READ);
  "LDO", "Return LDO voltage in mV | Return 0 if not available", "LDO", ldo_read, RAK_ATCMD_PERM_READ);
