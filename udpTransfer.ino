/*
 * wifi接続して、UDP通信をする。
 * 今回は９軸センサの情報を送る
 * 標準のM5.IMU.getAhrsData()はYaw軸（高さ方向）回転が勝手にドリフトするので使わず
 * Madgwickを使ってフィルター処理をかける
 * 地磁気センサーはbmm150。 
 * 
 * 起動時にPCや磁気が少なく水平な場所に制しておくこと（キャリブレーションする）
 * "running..."の時にM5Stackを適当にぐるぐる動かして磁気センサのオフセット値を取得する 
 *
 * 
 */
#define M5STACK_MPU6886
#include <WiFi.h>
#include <WiFiUDP.h>
#include <M5Stack.h>
#include <MadgwickAHRS.h>		//別途インストールする
#include "bmm150.h"
#include "bmm150_defs.h"

//const char ssid[] = "eoRT-106c12e-g";	//自宅
//const char pass[] = "****";

const char ssid[] = "SPWN_H36_01D34E";	//手持ちのポケットwifi
const char pass[] = "****";


WiFiUDP wifiUdp;
const char *pc_addr= "192.168.100.127"; //接続先IP
const int pc_port = 50007;  //送信先ポート
const int my_port = 50008;  //受信ポート今回は使わない

float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

Madgwick madgwick;  //姿勢計算フィルター

struct bmm150_dev dev;
bmm150_mag_data mag_offset;   //地磁気センサーのオフセット
bmm150_mag_data mag_max;        //地磁気センサーの最大値。 xyz
bmm150_mag_data mag_min;        //地磁気センサーの最小値。 xyz

//madgwickの計算周期 initialize variables to pace updates to correct rate
unsigned long microsPerReading, microsPrevious;

// the setup routine runs once when M5Stack starts up
void setup(){

  // Initialize the M5Stack object
  M5.begin();
  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();

  M5.IMU.Init();

  /***6軸センサーのキャリブレーション***/
  calibrate6886();  

  //地磁気センサーをチェック。I2C読めないと以上終了？
  if(bmm150_initialization() != BMM150_OK){
      M5.Lcd.println("BMM150 init failed");
      for(;;)
      {
          delay(100);
      }    
  }
  //磁気センサーをキャリブレーション
  bmm150_calibrate(10000);

  M5.Lcd.setTextSize(2);
  WiFi.begin(ssid,pass);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.println("WiFi connected");
  M5.Lcd.print("IP address = ");
  M5.Lcd.println(WiFi.localIP());

  madgwick.begin(50);  // 50Hzで計算。loopでこの頻度より早い周期で計算はさせない
  microsPerReading = 1000000 / 50;
  microsPrevious = micros();


  wifiUdp.begin(my_port);

}

#define INTERVAL 1    //平均値をとる回数
float rolls[INTERVAL],pitchs[INTERVAL],yaws[INTERVAL];
int times = 0;

// the loop routine runs over and over again forever
void loop() {
/*
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);   //yawの値がドリフトしてる。使えない
*/
 unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    //オフセットを反映させてセンサーを取得する
    applycalibration();
  
    //地磁気センサーで方向を取得。オフセットを指定。
    float head_dir = atan2(mag[0],mag[1]) * 180.0 / M_PI;
  
    
    //オフセット値を反映して姿勢情報を取得
    madgwick.update(gyro[0],gyro[1],gyro[2],
                       acc[0],acc[1],acc[2],
                       mag[0],mag[1],mag[2]);
    rolls[times] = madgwick.getRoll();
    pitchs[times] = madgwick.getPitch();
    yaws[times] = madgwick.getYaw();
  /*
    roll = 0;
    pitch= 0;
    yaw  = 0;
    for(int i=0; i<INTERVAL;i++){
      roll +=rolls[i];
      pitch+=pitchs[i];
      yaw  +=yaws[i];
    }
    
    roll  /= INTERVAL;
    pitch /= INTERVAL;
    yaw   /= INTERVAL;
  */
    M5.Lcd.println(".");
    
    //data send
    wifiUdp.beginPacket(pc_addr, pc_port);
  
    //表示
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("%4.2f %4.2f %4.2f",gyro[0],gyro[1],gyro[2]);
    
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("%4.2f %4.2f %4.2f",acc[0],acc[1],acc[2]);
  
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("%4.2f %4.2f %4.2f",mag[0],mag[1],mag[2]);
  
    M5.Lcd.setCursor(0, 90); 
    M5.Lcd.print(head_dir);

    M5.Lcd.setCursor(0, 120); 
    M5.Lcd.printf("%4.2f %4.2f %4.2f",roll,pitch,yaw);  

    
  //送信
    String str;  
    str = String(roll, 2)+","+String(pitch, 2)+","+String(yaw, 2);    
    byte msg[str.length()+1];   //最後がエンドコードになるせいか、受信側で１ビット足りないので追加する
    str.getBytes(msg,str.length()+1);
    wifiUdp.write(msg,str.length()+1);    

  /*   wifiUdp.write(roll,sizeof(float));
    wifiUdp.write(yaw,sizeof(float));*/
    wifiUdp.endPacket();
  
    //平均値計算用のカウント
    times++;
    if(times >= INTERVAL){ times = 0;}
    roll =0;
    pitch=0;
    yaw = 0; 
    for(int i=0; i<INTERVAL; i++){
      roll += rolls[i];
      pitch+= pitchs[i];
      yaw  += yaws[i];
    }
    roll /= INTERVAL;
    pitch/= INTERVAL;
    yaw  /= INTERVAL;

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

/*
 * センサーデータを補正
 */
void applycalibration(){
  M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
  M5.IMU.getAccelData(&acc[0],&acc[1],&acc[2]);
  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];

  bmm150_read_mag_data(&dev);
  mag[0] = dev.data.x - mag_offset.x;
  mag[1] = dev.data.y - mag_offset.y;
  mag[2] = dev.data.z - mag_offset.z;
}

/*地磁気のキャリブレーション*/
/* counter数回回して平均値をとる
 * これによって動いている状態(特にyawの加速度を軽減させる) 
 */
void calibrate6886(){
  float gyroSum[3];
  float accSum[3];
  int counter = 500;
  
  M5.Lcd.println("IMU 6866 calibration...");
  
  for(int i = 0; i < counter; i++){
    M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
    M5.IMU.getAccelData(&acc[0],&acc[1],&acc[2]);
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    delay(2);
  }
  gyroOffset[0] = gyroSum[0]/counter;
  gyroOffset[1] = gyroSum[1]/counter;
  gyroOffset[2] = gyroSum[2]/counter;
  accOffset[0] = accSum[0]/counter;
  accOffset[1] = accSum[1]/counter;
  accOffset[2] = (accSum[2]/counter) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定

  M5.Lcd.println("Finish");
}


/*
 * 地磁気センサーの初期化。I2Cの読み書きチェックをして
 * 正常だったらBMM150_OKを返す
 */
int8_t bmm150_initialization()
{
    int8_t rslt = BMM150_OK;

    /* Sensor interface over SPI with native chip select line */
    dev.dev_id = 0x10;
    dev.intf = BMM150_I2C_INTF;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_ms = delay;

    /* make sure max < mag data first  */
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    /* make sure min > mag data first  */
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&dev);
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(&dev);
    return rslt;
}

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    if(M5.I2C.readBytes(dev_id, reg_addr, len, read_data))
    {
        return BMM150_OK;
    }
    else
    {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    if(M5.I2C.writeBytes(dev_id, reg_addr, read_data, len))
    {
        return BMM150_OK;
    }
    else
    {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

/*
 * 地磁気センサーのキャリブレーション。
 * グルグル回している時に、最大値と最小値を取ることでオフセット値とする
 * シグナル発生してからまた鳴るまでの間作業する
 */
void bmm150_calibrate(uint32_t calibrate_time)
{
    uint32_t calibrate_timeout = 0;

    calibrate_timeout = millis() + calibrate_time;
    M5.Lcd.printf("Go calibrate, use %d ms \r\n", calibrate_time);
    M5.Lcd.printf("running ...");

    while (calibrate_timeout > millis())
    {
        bmm150_read_mag_data(&dev);
        if(dev.data.x)
        {
            mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
            mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
        }

        if(dev.data.y)
        {
            mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
            mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
        }

        if(dev.data.z)
        {
            mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
            mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
        }
        delay(100);
    }

    mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;

    M5.Lcd.printf("calibrate finish ... \r\n");
    M5.Lcd.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
    M5.Lcd.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
    M5.Lcd.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}
