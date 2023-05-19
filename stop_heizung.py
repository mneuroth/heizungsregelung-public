
try:
    import urllib2
except ModuleNotFoundError as exc:
    import urllib.request as urllib2

def stop_heizung():
    response = urllib2.urlopen("http://127.0.0.1/stop")
    html = response.read()
    return html

print(stop_heizung())
