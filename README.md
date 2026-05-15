Tabela — Single Core:

Métrica / Task	mpu_task	fusion_task	uart_task	pwm_task
- WCET
  
mpu_task: 10.003
fusion_task: 100.005
uart_task: 49.995
pwm_task: 240 

- Deadline Miss Rate			

mpu_task: 100%
fusion_task: 100%
uart_task: 0%
pwm_task: 100%

- Stack Usage				

mpu_task: 1.8%
fusion_task: 38.0%
uart_task: 42.9%
pwm_task: 35.1%


Tabela — SMP (2 cores):

- WCET
  
mpu_task: 63.2 micro
fusion_task: 21.52 micro
uart_task: 2.92 micro
pwm_task: 1.2 micro

- Deadline Miss Rate			

mpu_task: 0%
fusion_task: 0%
uart_task: 0%
pwm_task: 0%

- Stack Usage				

mpu_task: 55.4%
fusion_task: 75.2%
uart_task: 57.2%
pwm_task: 53.1%
