import urllib

f = urllib.urlopen("http://127.0.0.1/stop")
#f = urllib.urlopen("http://127.0.0.1:8000/;stop")
s = f.read()
print(s)
