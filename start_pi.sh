cd ~/Documents/projects/heizungsregelung
lxterminal --command="/bin/bash --init-file '~/Documents/projects/heizungsregelung/strompi2/start_watchdog.sh'"
netstat -a | grep 42424
while [ $? -eq 0 ]
do
    python -c "import time; print('waiting for available port 42424 (heizung_exporter)...'); time.sleep(5)"
    netstat -a | grep 42424
done
lxterminal --command="/bin/bash --init-file '~/Documents/projects/heizungsregelung/start_exporter.sh'"
#netstat -a | grep 42425
#while [ $? -eq 0 ]
#do
#    python -c "import time; print('waiting for available port 42425 (pv_facility_exporter)...'); time.sleep(5)"
#    netstat -a | grep 42425
#done
lxterminal --command="/bin/bash --init-file '~/Documents/projects/heizungsregelung/start_pv_exporter.sh'"
#./start_chromium.sh &
./start_firefox.sh &
while true
do
   echo "============> starting heating control <=============="
   sudo python heizung.py -s -w -p
   #test: sudo python heizung.py -t -i
   echo "------------> heating control stoped ! <--------------"
   python -c "import time; time.sleep(15)"
done
