# Project Pick & Place voor TNO
Dit is een project met een Pick & Place cobot. Er is een simulatie voor zowel de ur5 als de panda van Franka Emika.


# Om dit project te openen
Zorg dat de volgende afhankelijkheden geinstalleerd zijn:
- ros-kinetic-full

Clone dit project in de src folder van een catkin_workspace en make.


# Gewrichten limieten
Gewricht | min rad | max rad | min deg | max deg 
--- | --- | --- | --- |---
Shoulder pan | -1.3 | 1.3 | -74.48 | 74.48
Shoulder lift | -1.7453 | -0.5235 | -100 | -30
Elbow joint | x | 3.15 | x | 180.48
Wrist1 | 2.356 | 6.28 | 134.99 | 359.82
Wrist2 | -3.15 | 3.15 | -180.48 | 180.48
Wrist3 | -3.15 | 3.15 | -180.48 | 180.48

Deze limieten hebben meerdere functies. De belangrijkste functie is dat het de oplosruimte verkleint, wat tot gevolg heeft dat de robot op een veilige manier van a naar b gaat. Een andere bijwerking van het verkleinen van de oplosruimte is dat het minder rekenkracht kost om een weg voor de robot te berekenen.
Let op, als de robot in een positie staat die niet binnen de hierboven genoemde limieten vallen zal de planner geen weg kunnen plannen. Zorg dus altijd dat de startpositie binnen de limieten valt.

# Resultaten
youtu.be/bivQM2FoDJs


## Autheurs:
- Martijn Schoenmaeker
- Dirk Brouwers
- Maik Sigmans
- Niels van den Bos
