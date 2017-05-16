# Arduino MIDI controller

## Аннотация	

Автор проекта: Руслан Фаткуллин, CSC, 2017.

Идея проекта - создание прототипа устройства способного передавать сообщения по MIDI (Musical Instrument Digital Interface) протоколу для внешних MIDI совместимых устройств (звуковые карты, различные звуковое оборудование - поддерживающее midi протокол). 

Возможное усовершенствование - использование дополнительного адаптера (bluetooth) для беспроводной передачи.

## Пример исполнения для прямого подключения к midi устройству

![alt tag](https://github.com/fatkullin/arduino/blob/master/IMG_20170515_190513.jpg)


## Требуемый контроллер
Arduino Uno (плюс ещё один, если будет использоваться беспроводная передача сигналов)

## Периферийное оборудование 
* Основное

 3 шт тактовые кнопки (tactile pushbutton switches)

 3 шт 10K ohm резистор
 
 3 шт переменных резистора (потенциометра) (10K rotary potentiometer)
 
* Вывод сигнала 

 1 шт Female MIDI jack (5-pin DIN, другие названия СШ-5, СГ-5, DIN 41524, «5-pin DIN 180°», DIN-5/180°.)

 2 шт 220 ohm resistors


* Для беспроводной передачи 

 2 блютус модуля (например HC-05 или HC-06)

 2 шт 1K ohm резистор 

 2 шт 2.2K ohm резистор 

* Провода, монтажные площадки для прототипирования

## Среда разработки:
 MS Visual Studio 2015 + Visual Micro 


## Схема c использованием bluetooth для беспроводной передачи сигнала

![alt tag](https://github.com/fatkullin/arduino/blob/master/midi-blue%20main%20scheme1.png)

![alt tag](https://github.com/fatkullin/arduino/blob/master/midi-blue%20main%20scheme2.png)

## Breadboard

![alt tag](https://github.com/fatkullin/arduino/blob/master/midi-blue%20main%20bb1.png)

![alt tag](https://github.com/fatkullin/arduino/blob/master/midi-blue%20main%20bb2.png)

## Datasheets

HC-06
https://www.olimex.com/Products/Components/RF/BLUETOOTH-SERIAL-HC-06/resources/hc06.pdf

## Библиотеки
SoftwareSerial
https://www.arduino.cc/en/Reference/softwareSerial
