#ifdef __cplusplus
extern "C" {
#endif

extern int mpu6050Setup(const int i2cAddress/* = 0x68*/);
extern int readData(int address);

extern float mpu6050ReadAccX();
extern float mpu6050ReadAccY();
extern float mpu6050ReadAccZ();

extern float mpu6050ReadGyroX();
extern float mpu6050ReadGyroY();
extern float mpu6050ReadGyroZ();

extern int mpu6050IsConnected();
extern void mpu6050GyroOffset();
extern void mpu6050Update();


#ifdef __cplusplus
}
#endif
