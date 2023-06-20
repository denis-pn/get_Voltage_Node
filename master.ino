/*
   Мастер создаёт сетевую таблицу с номерами узлов и их состоянием.
   nodeID записывается на позицию соответсвующую его номеру с 0 по 9 (можно расширить).
   Во втором столбце таблицы указывается активность: 0 - узел не активен, 1 - активен.
   Если в обоих столбцах 0, значит такого узла в сети ещё не было.
   Если произошла смена мастера, то новый мастер ставит на свою бывшую позицию в первом столбце 0.
   При этом во втором столбце остаётся 1, это позволяет не терять информацю.

   Префиксы сообщений:
   H - сетевая таблица состояния
   V - служебная информация (номер канала для смены)
   T - Информация
   S - тестовые для подсчёта метрик
   N - старт измерений
   M - запрос данных эксперимента
   С - для проверки связи
   Максимальная длина сообщения 120 байт (uint8_t до 255)

   Частоты: 2425, 2450, 2475, 2478 и 2481 МГц, мощность не более 100 мВт
*/

#include <AESLib.h>
#include <Arduino.h>
//#include <unistd.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>
#include "nRF24L01.h"

RF24 radio(1, 0);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

// переменные для функции выбора канала
uint8_t servise_channel = 70;// канал для радио
uint8_t values[83];//массив для функции сканера
uint8_t key_0[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; //ключ для шифрования частоты

//общие переменные
uint8_t nodeID = 0;
uint8_t static_IP[4] = {192, 168, 10, nodeID}; //последний октет совпадает с nodeID присвоенным изначально
uint8_t channel = 81; // несущая для сети по умолчанию 81
uint8_t netTable[10][2]; // сетевая таблица для учёта смены узла
uint32_t fails;
uint32_t success;
uint16_t d_fails;
uint16_t d_success;

//переменные при работе в качестве мастера
uint8_t nodeAmount = 2; // общее количество активных узлов в сети включая мастера
uint16_t failID = 0;
uint32_t del = 200;//время задержки отправки тестовых сообщений
uint32_t dataTable[10][6];// [0] - pdr up, [1] - pdr down, iptd up, iptd down, ipdv up, ipdv down
uint32_t dataCF[3]; //данные эксперимента от узлов
uint8_t n; //nodeID для передачи
uint8_t n1; //количество активных узлов в сети
char symb;
uint8_t data[] = {255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0};
uint8_t dataR[32];
uint8_t k; // счётчик количества посылаемых тестовых сообщений (в программе 10)
uint8_t ID; //nodeID на приём
uint32_t pdr;
uint32_t t[10]; // массив времён передачи для рассчёта IPTD
uint32_t IPTD[9];// массив для рассчёта IPTD
uint32_t a_IPTD;// средний IPTD
uint32_t m_IPTD;// минимальный IPTD
uint32_t IPDV;

bool p0 = 0; // переключатель radio/mesh
bool p1 = 0;//переклюатель на передачу тестовых сообщений S типа
bool p2; // активатор тестирования pdr iptd ipdv
bool p3; //переключатель для активации таймаута
bool p4 = 1; // активатор функции пингования
bool xn = false; //сигнал об окончании передачи узлом
bool x0 = false; //сигнал об окончании передачи мастером

// различные переменные
uint32_t displayTimer0 = 0;// таймер пингования (master)/отправки сообщений(node)
uint32_t displayTimer1 = 0; //время задержки отправки сообщений S типа
uint32_t displayTimer2 = 0; //таймаут при тестировании узлов (master)
uint32_t displayTimer3 = 0; // для отправки по radio текущего канала (node)
uint32_t displayTimer4 = 0; //таймер проверки наличия хотя бы одного активного узла в сети
//переменные для шифрования
//длина блока должна быть 128 бит, размер символа 8 бит
char route[16];
uint8_t key[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t dataE[16];

float voltage = 0;
uint16_t batteryLevel;

void setup() {
  delay(5000);
  Serial.begin(9600);
  Serial.print(static_IP[0]); Serial.print("."); Serial.print(static_IP[1]); Serial.print("."); Serial.print(static_IP[2]); Serial.print("."); Serial.println(static_IP[3]);
}
//-------------функция пингования--------------------------------------------
void pingNode(uint8_t listNo) {
  RF24NetworkHeader headers(mesh.addrList[listNo].address, NETWORK_PING);
  uint32_t pingtime = millis();
  bool ok;
  if (headers.to_node) {
    ok = network.write(headers, 0, 0);
    if (ok && failID == mesh.addrList[listNo].nodeID) {
      failID = 0;
    }
    if (!ok) {
      failID = mesh.addrList[listNo].nodeID;
    }
  }
  pingtime = millis() - pingtime;
  Serial.print(" Time: "); Serial.print(pingtime); Serial.print(" ms; ");
  netTable[0][1] = 1;//мастер подразумеваем включённым
  if (ok || !headers.to_node) {
    Serial.println("status ONLINE");
    netTable[mesh.addrList[listNo].nodeID][0] = mesh.addrList[listNo].nodeID;
    netTable[mesh.addrList[listNo].nodeID][1] = 1;
  } else {
    Serial.println("status OFFLINE");
    // netTable[mesh.addrList[listNo].nodeID][0] = 0;// если узел отключен замени в таблице на 0
    netTable[mesh.addrList[listNo].nodeID][1] = 0;
  }
}

//-------------------------------------------------------------------------------------------------------
void loop() {
  if (p0 == 0) {
    //------запускаем MESH----------//----------------------------
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    mesh.setNodeID(0);
    mesh.begin(channel, RF24_250KBPS, MESH_RENEWAL_TIMEOUT);
    mesh.update();
    mesh.DHCP();
    Serial.println("канал установлен, mesh запущен");
    p0 = 1;
  }
  //-----------------------------------Работа mesh---------------------------------------------
  else {
    mesh.update();
    mesh.DHCP();
    //-----------------------------Чтение команд с клавиатуры------------------------------------------------
    if (Serial.available()) {
      symb = Serial.read();
      if (symb == 't') { // сигнал о начале тестирования
        p4 = 0;// отключаем функцию пингования
        p2 = 1;// включаем функцию отслеживания
        n = 1; // переключатель номера запрашиваемого узла (начинаем с 1)
        x0 = false; // сигнал об окончании передачи со стороны мастера
        xn = false; // сигнал об окончании передачи со стороны узла
        for (uint8_t i = 0; i < 6; i++) {
          for (uint8_t j = 0; j < nodeAmount; j++) { // очистка таблицы данных
            dataTable[i][j] = 0;
          }
        }
        Serial.println("Tests start");
      }
    }
 
    //-----------------------------------------------------------------------------------------------
    //----------------------Приём сообщений----------------------------------------------------------
    if (network.available()) {
      RF24NetworkHeader header;
      network.peek(header);
      ID = mesh.getNodeID(header.from_node);

      if (header.type == 'S') { // рассчёт характеристик при получении тестового сообщения
        network.read(header, &dataR, sizeof(dataR));
        if (pdr < 10) {
          t[pdr] = millis();
        }
        pdr++;
      }
      else if (header.type == 'N') { // сообщение об окончании отправки тестовых сообщений узлом
        network.read(header, &dataR, sizeof(dataR));
        xn = true;
        Serial.print("node "); Serial.print(ID); Serial.println(" has finished test");
      }
      else if (header.type == 'M') { // запись характеристик pdr iptd ipdv от узлов в таблицу
        network.read(header, &dataCF, sizeof(dataCF));
        dataTable[3][ID] = dataCF[0];
        dataTable[4][ID] = dataCF[1];
        dataTable[5][ID] = dataCF[2];
        n++;
        p2 = 1;
        if (n > nodeAmount) { //когда пробежали по всей netTable и получили послений ответ заканчваем процедуру
          n = 0;
          p2 = 0;
          p3 = 0;
        }
        else {
          p2 = 1;
          p3 = 0;
        }
      }
      else if (header.type == 'T') {
        network.read(header, &voltage, sizeof(voltage)); //приняли зашифрованное сообщение
        /*Serial.print("Encripted message: ");
          for (uint8_t i = 0; i < 16; i++) {
          Serial.print(dataE[i]); Serial.print(" ");
          }
          Serial.println();*/
//        aes128_dec_single(key, dataE);//дешифруем сообщение
        Serial.print("From NodeID "); Serial.println(ID);
        Serial.print("Decripted message: ");
//        for (uint8_t i = 0; i < 16; i++) { //переводим из типа uint8_t в необходимы тип char
//          route[i] = dataE[i];
//          Serial.print(route[i]); Serial.print(" ");
//        }
        Serial.print(voltage); Serial.print(" ");
        Serial.println();
      }
      else if (header.type == 'C') {
        network.read(header, &dataE, sizeof(dataE)); //приняли зашифрованное сообщение для проверки связи
      }
    }
    //-----------------------------------------------------------


    //-----------вывод состояния сети---------------------------------
    if ((millis() - displayTimer0 > 5000) && (p4 == 1)) {
      mesh.update();
      mesh.DHCP();
      displayTimer0 = millis();
      Serial.println(" ");
      Serial.println(F("********Assigned Addresses********"));
      for (int i = 0; i < mesh.addrListTop; i++) {
        Serial.print("NodeID: ");
        Serial.print(mesh.addrList[i].nodeID);
        Serial.print("; ");
        Serial.print("RF24Network Address: ");
        Serial.print(mesh.addrList[i].address, OCT);
        Serial.print("0; ");
        pingNode(i);
      }
      for (int i = 0; i < nodeAmount; i++) { // показ составленной сетевой таблицы
        Serial.print(netTable[i][0]);
        Serial.println(netTable[i][1]);
      }
      Serial.println(F("**********************************"));
      for (int i = 0; i < nodeAmount; i++) { // отправка таблицы на узлы
        if (netTable[i][1] == 1 && netTable[i][0] != 0) {
          mesh.write(&netTable, 'H', sizeof(netTable), netTable[i][0]);
          // Serial.print(netTable[i][0]);
          // Serial.println(" таблица отправлена");
        }
      }
    }
    //-----------------------//-------------------------------------------
  }
}
