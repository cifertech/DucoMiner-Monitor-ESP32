<div align="center">

  <img src="https://user-images.githubusercontent.com/62047147/195847997-97553030-3b79-4643-9f2c-1f04bba6b989.png" alt="logo" width="100" height="auto" />
  <h1>DucoMiner Monitor ESP32</h1>
  
  <p>
    Mine DuinoCoin and Monitor Details with ESP32
  </p>
  
  
<!-- Badges -->

<p>
<a href="https://github.com/cifertech/DucoMiner-Monitor-ESP32" title="Go to GitHub repo"><img src="https://img.shields.io/static/v1?label=cifertech&message=DucoMiner-Monitor-ESP32&color=white&logo=github" alt="cifertech - DucoMiner-Monitor-ESP32"></a>
<a href="https://github.com/cifertech/DucoMiner-Monitor-ESP32"><img src="https://img.shields.io/github/stars/cifertech/DucoMiner-Monitor-ESP32?style=social" alt="stars - DucoMiner-Monitor-ESP32"></a>
<a href="https://github.com/cifertech/DucoMiner-Monitor-ESP32"><img src="https://img.shields.io/github/forks/cifertech/DucoMiner-Monitor-ESP32?style=social" alt="forks - DucoMiner-Monitor-ESP32"></a>
   
<h4>
    <a href="https://twitter.com/cifertech1">TWITTER</a>
  <span> · </span>
    <a href="https://www.instagram.com/cifertech/">INSTAGRAM</a>
  <span> · </span>
    <a href="https://www.youtube.com/c/cifertech">YOUTUBE</a>
  <span> · </span>
    <a href="https://cifertech.net/">WEBSITE</a>
  </h4>
</div>

<br />

<!-- Table of Contents -->
# :notebook_with_decorative_cover: Table of Contents

- [About the Project](#star2-about-the-project)
  * [Pictures](#camera-Pictures)
  * [Features](#dart-features)
- [Getting Started](#toolbox-getting-started)
  * [Schematic](#electric_plug-Schematic)
  * [Installation](#gear-installation)
- [Usage](#eyes-usage)
- [Contributing](#wave-contributing)
- [License](#warning-license)
- [Contact](#handshake-contact)

  

<!-- About the Project -->
## :star2: About the Project
A few months ago, we published a tutorial on extracting a currency called DuinoCoin. Now we performed the extraction operation with the help of ESP32 and with OLED, we displays Walt inventory values, number of miners, and various values at the same time as mining.

<!-- Pictures -->
### :camera: Pictures

<div align="center"> 
  <img src="https://user-images.githubusercontent.com/62047147/195975884-94b190ca-093f-4e69-8607-b8aa623f2467.jpg" alt="screenshot" />
</div>

<!-- Features -->
### :dart: Features

- Show wallet balance
- Display the number of miners
- Mine at the same time as displaying wallet information

<!-- Getting Started -->
## 	:toolbox: Getting Started

We will use ESP32 as a processor. and we will add an OLED displays Walt inventory values, number of miners, and various values at the same time as mining.

- ESP32
- Oled 0.91 SSD1306

<!-- Schematic -->
### :electric_plug: Schematic

We have the pins related to the I2C protocol to communicate between the OLED display and the ESP32 board. In the ESP32 board, we will use pins D21, and D22, and in the OLED display, SCL and SDA pins. We will also use VIN or 5V pins on the ESP32 board for the power supply.

Make the connections according to the table and schematic below.

| ESP32| Oled 0.96|
| ----   | -----|
| D22 | SCL |
| D21 | SDA |
| 5v  | VDD |
| GND | GND |


* Complete Schematic

<img src="https://user-images.githubusercontent.com/62047147/195975926-297d43b6-742c-4837-b4ce-c0ede36a34ad.jpg" alt="screenshot" width="800" height="auto" />


<!-- Installation -->
### :gear: Installation

Change the following according to your Wi-Fi network information and your Doinocoin account name.
```c++
const char *wifi_ssid = "C1F3R";
const char *wifi_password = "314159265";
const char *username = "CiferTech";
```

This is the case for your ESP32 chip version. If you have a serious problem uploading the code, comment on the first line and remove the second line from the comment.
```c++
#include "hwcrypto/sha.h"
//#include "sha/sha_parallel_engine.h"
```
   
   
   
<!-- Usage -->
## :eyes: Usage

Finally, after changing the information mentioned in the code, establishing connections and finally uploading the code, the values in the wallet, the number of miners and the estimated amount of mines based on the number of miners and HASH RATE are displayed in the OLED display, values after a short period They are updated according to the time of mining your board and changing the received information.

<!-- Contributing -->
## :wave: Contributing

<a href="https://github.com/cifertech/DucoMiner-Monitor-ESP32/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cifertech/DucoMiner-Monitor-ESP32" />
</a>


<!-- License -->
## :warning: License

Distributed under the MIT License. See LICENSE.txt for more information.


<!-- Contact -->
## :handshake: Contact

CiferTech - [@twitter](https://twitter.com/cifertech1) - CiferTech@gmali.com

Project Link: [https://github.com/cifertech/DucoMiner-Monitor-ESP32](https://github.com/cifertech/DucoMiner-Monitor-ESP32)

<!-- Acknowledgments -->
## :gem: Acknowledgements

Special Thanks to ...

 - [ponsato](https://github.com/ponsato)


