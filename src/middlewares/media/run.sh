ICON=header/icon.h

rm $ICON

                       #1: Image file     #2: Variable      #3: Generate file name
python3 make_header.py images/battery_level_0.png   battery_level_0             $ICON
python3 make_header.py images/battery_level_25.png  battery_level_25            $ICON
python3 make_header.py images/battery_level_50.png  battery_level_50            $ICON
python3 make_header.py images/battery_level_75.png  battery_level_75            $ICON
python3 make_header.py images/battery_level_100.png battery_level_100           $ICON

python3 make_header.py images/ble.png ble                                       $ICON
python3 make_header.py images/ble_active.png ble_active                         $ICON

python3 make_header.py images/bolt.png bolt                                     $ICON
python3 make_header.py images/check_circle.png check_circle                     $ICON
python3 make_header.py images/circle.png circle                                 $ICON
python3 make_header.py images/heart_outline.png heart_outline                   $ICON
python3 make_header.py images/heart_solid.png heart_solid                       $ICON
python3 make_header.py images/vector.png vector                                 $ICON

python3 make_header.py images/signal_acq_1.png signal_acq_1                     $ICON
python3 make_header.py images/signal_acq_2.png signal_acq_2                     $ICON

python3 make_header.py images/temperature.png temperature                       $ICON

python3 make_header.py images/zigbee_signal_0.png zigbee_signal_0               $ICON
python3 make_header.py images/zigbee_signal_1.png zigbee_signal_1               $ICON
python3 make_header.py images/zigbee_signal_2.png zigbee_signal_2               $ICON
python3 make_header.py images/zigbee_signal_3.png zigbee_signal_3               $ICON
python3 make_header.py images/zigbee_signal_4.png zigbee_signal_4               $ICON
python3 make_header.py images/clear_12_12.png clear_12_12                       $ICON
python3 make_header.py images/clear_24_24.png clear_24_24                       $ICON


