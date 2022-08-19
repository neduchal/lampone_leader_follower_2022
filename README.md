# Campo Lampone 2022 

## Úloha Leader Follower 

Tento repozitář obsahuje zadání hlavní úlohy letního robotického soustředění Campo Lampone 2022. Tématem úlohy je úloha nazvaná Leader Follower, kde jde o to, aby tým implementoval kód, který po nahrání do robota (Followera) způsobí, že robot bude následovat robota (Leadera) pomocí jeho obrazové detekce.

### Motivace

K čemu něco takového může být? Co například částečně automatizovaný převoz nákladu v rámci nějaké oblasti (sklad, areál firmy, silnice mezi dvěma městy). V takovém případě by stačil jeden manuálně řízený robot/vozidlo. Zbytek robotů/vozidel pak jede automaticky a následuje vedoucího robota/vozidlo).

### Základní informace

* Kód úlohy je k nalezení v souboru src/leader_follower.py v tomto repozitáři.
* Úloha využívá fimrware dostupný v repozitáři https://github.com/neduchal/lampo_firmware 
* Na začátku soustředění budou mít týmy k dispozici robota s přednahraným softwarem. V případě změn v repozitářích lampo_firmware nebo lampone_leader_follower_2022 je potřeba v patřičném adresáři zavolat příkaz ** git pull **
* Zatím se předpokládá využití robotů založených na Raspberry Pi4. V přípravě je též firmware na roboty s počítačem NVidia Jetson Nano -- bude k dispozici v brzké době.

### Pokyny:

* Implementujte funkce v souboru leader_follower.py (bude součástí přednášky v první den soustředění)
* Během řešení úlohy budou k dispozici trénovací roboti typu mBot osazení vzadu červeným terčem a QR kódem. Na něm je možné testovat řešení v průběhu soustředění.
* Týmy se následně zúčastní soutěže. Hlavním kritériem je doba, po kterou se dokážou udržet za leaderem v určitém rozmezí vzdáleností. 
* Pokud se vše vydaří bude cílem posledního dne vytvořit z followerů vláček. 

