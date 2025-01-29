#/bin/bash -e
echo "Patching generated syscfg files:"

echo "- Disabling DCDC"
sed -i 's/#define SET_CCFG_MODE_CONF_DCDC_RECHARGE                0x0/#define SET_CCFG_MODE_CONF_DCDC_RECHARGE 0x1/g' syscfg/ti_devices_config.c
sed -i 's/#define SET_CCFG_MODE_CONF_DCDC_ACTIVE                  0x0/#define SET_CCFG_MODE_CONF_DCDC_ACTIVE 0x1/g' syscfg/ti_devices_config.c


if [ "$CC1352P_2_OTHER" = "true" ]; then
    echo "- CC1352P_2_OTHER changes"
    sed -i 's/#define CONFIG_RF_24GHZ 28/#define CONFIG_RF_24GHZ 6/g' syscfg/ti_drivers_config.h
    sed -i 's/#define CONFIG_RF_HIGH_PA 29/#define CONFIG_RF_HIGH_PA 5/g' syscfg/ti_drivers_config.h
    sed -i 's/#define SET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA          0xc1/#define SET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA 0xfa/g' syscfg/ti_devices_config.c
fi
