while true
do
  netstat -a | grep 42425
  while [ $? -eq 0 ]
  do
      python -c "import time; print('waiting for available port 42425 (pv_facility_exporter)...'); time.sleep(5)"
      netstat -a | grep 42425
  done
  sleep 15
  /home/pi/Documents/projects/heizung_pyenv/bin/python ~/Documents/projects/heizungsregelung/pv_facility_exporter.py
done
