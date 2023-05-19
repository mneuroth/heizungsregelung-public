## Allgemeine Einleitung für die Heizungsregelung

Ansteuerung
-----------

Die Ansteuerung kann über verschiedene Wege erfolgen:

- auf dem Raspberry Pi direkt, über den angeschlossenen Touchscreen -> Browser öffnen und `http://localhost` eingeben.
- von einem beliebigen Computer oder Handy im Haus WLAN -> Browser öffnen und `http://heizungsregelung/` eingeben.

Die Dokumentation der Heizungsregelung gibt es im Internet auf GitHub:

- [GitHub Projekt Page auf https://github.com/mneuroth/heizungsregelung-public](https://github.com/mneuroth/heizungsregelung-public)

Wichtige Daten:

- IP-Adresse vom Raspberry Pi: `___.___.___.___`
- Root Zugriff auf Raspberry Pi: 
    - user: `              `
	- password: `              `
- Quellcode für Regelung und Firmware für Heizungsregelung-Platine auf GitHub: https://github.com/mneuroth/heizungsregelung-public
- GitHub User 
    - user: `              `
	- password: `              `
	
----	

Einstellungen der Zeitbereiche
------------------------------

Strom für Wärmepumpe | Zeitbereich | Tag
------------- | -------------- | --------
ausgeschaltet | 11:30 -> 13:00 | Mo - Fr
ausgeschaltet | 17:30 -> 19:00 | Mo - Fr

Heizung Umwälzpumpe | Zeitbereich | Tag  | Bemerkung
------------- | -------------- | ------- | ------
eingeschaltet | 8:00 -> 14:00  | Mo - Fr | 1. November - 1. März
eingeschaltet | 9:00 -> 17:00  | Sa, So  | 1. November - 1. März
eingeschaltet | 21:00 -> 3:00  | Mo - So | 1. November - 1. März
eingeschaltet | 22:30 -> 22:35 | Mo - So | jeden Tag wegen Anti-Festlauf

Mixer schließen | Zeit | Tag
--------------- | ---- | -----
schließen | 14:00 | Mo - So
schließen | 17:00 | Mo - So
schließen | 13:00 | Mo - So
	