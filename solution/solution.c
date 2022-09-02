#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#define TIME_STEP 48
#define V_MAX 10.0
#define L 0.2 //largura carrinho
#define r 0.035 //raio das rodas
#define tol 0.0 //tolerancia
#define lim 15.0 //limite de correcao
#define comptime 0.5 //tempo de compensacao

//Definindo variaveis gerais:
WbDeviceTag MTD, MTE, LFE, LFCE, LFC, LFCD, LFD, ENC_D, ENC_E, USE, USC, USD;
float LCD, LCE, LC, LE, LD, lue, luc, lud;
double c, som;

void delay(double duration)
{
    double time = wb_robot_get_time();
    while (wb_robot_step(TIME_STEP) != -1) 
    {
        if ((wb_robot_get_time()-time) >= duration)
        {
            break;
        }        
    }    
}

void andar(float meters, float velocity)
{
    wb_motor_set_velocity(MTD, velocity);
    wb_motor_set_velocity(MTE, (velocity));
    double P0 = wb_position_sensor_get_value(ENC_D);
    while (wb_robot_step(TIME_STEP) != -1) 
    {
        double ED = wb_position_sensor_get_value(ENC_D);
        if ((ED - P0) >= (meters/r))
        {
            break;
        }               
    }  
}

void curve(float degrees, float ray, float velocity)
{
    degrees = ((degrees/180.0)*M_PI);
    float k = ((float)((2.0*ray)-L)/((2.0*ray)+L));
    if (degrees > 0)
    {
        wb_motor_set_velocity(MTD, velocity);
        wb_motor_set_velocity(MTE, (velocity*k));
        double P0 = wb_position_sensor_get_value(ENC_D);
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            double ED = wb_position_sensor_get_value(ENC_D);
            if ((ED - P0) >= ((fabs(degrees)*((2.0*ray)+L))/(2*r)))
            {
                break;
            }               
        }  
    } else
    {
        wb_motor_set_velocity(MTD, (velocity*k));
        wb_motor_set_velocity(MTE, velocity);
        double P0 = wb_position_sensor_get_value(ENC_E);
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            double EE = wb_position_sensor_get_value(ENC_E);
            if ((EE - P0) >= ((fabs(degrees)*((2.0*ray)+L))/(2*r)))
            {
                break;
            }                 
        }  
    }    
    wb_robot_step(TIME_STEP);
    wb_motor_set_velocity(MTD, 0.0);
    wb_motor_set_velocity(MTE, 0.0);   
}

void linecurve(char direction, float ray, float velocity)
{
    float k = ((float)((2.0*ray)-L)/((2.0*ray)+L));
    if (direction == 'E')
    {
        wb_motor_set_velocity(MTD, velocity);
        wb_motor_set_velocity(MTE, (velocity*k));
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            LC = wb_distance_sensor_get_value(LFC);
            if (LC > 130)
            {
                break;
            }               
        }  
    } else
    {
        wb_motor_set_velocity(MTD, (velocity*k));
        wb_motor_set_velocity(MTE, velocity);
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            LC = wb_distance_sensor_get_value(LFC);
            if (LC > 130)
            {
                break;
            }               
        }  
    }    
    wb_robot_step(TIME_STEP);
    wb_motor_set_velocity(MTD, 0.0);
    wb_motor_set_velocity(MTE, 0.0);   
}

int nonclockwise_seek(char side, int skip)
{
    lue = wb_distance_sensor_get_value(USE);
    luc = wb_distance_sensor_get_value(USC);
    lud = wb_distance_sensor_get_value(USD);
    if (side == 'D')
    {
        if(lud < 1000)
        {
            //Já no lado certo
            wb_motor_set_velocity(MTD, 10.0);
            wb_motor_set_velocity(MTE, 9.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                lud = wb_distance_sensor_get_value(USD);
                if (lud < 800)
                {
                    break;
                }                      
            }
            wb_motor_set_velocity(MTD, 0.0);
            wb_motor_set_velocity(MTE, 0.0);
        } else
        {
            //vai para o lado indicado:
            wb_motor_set_velocity(MTD, 5.0);
            wb_motor_set_velocity(MTE, 10.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                luc = wb_distance_sensor_get_value(USC);
                lud = wb_distance_sensor_get_value(USD);
                if ((luc < 800) || (lud < 800))
                {
                    break;
                }                      
            }
            //Alinhar
            wb_motor_set_velocity(MTD, 10.0);
            wb_motor_set_velocity(MTE, 1.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                lud = wb_distance_sensor_get_value(USD);
                if (lud < 800)
                {
                    break;
                }                      
            }
        }
        long long int cont=0;
        //aprende a circunferência do labirinto
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            lud = wb_distance_sensor_get_value(USD);
            if(lud >= 1000)
            {
              if(skip == 0)
              {
                curve(-90, 0.1, V_MAX);
              }else
              {
                wb_motor_set_velocity(MTD, V_MAX);
                wb_motor_set_velocity(MTE, (V_MAX*(c/cont)));
                while (wb_robot_step(TIME_STEP) != -1) 
                {
                  lud = wb_distance_sensor_get_value(USD);
                  if(lud < 1000)
                  {
                    break;
                  }
                }
                skip--;
                continue;
              }
            }
            float k = ((900-lud)/100.0);
            c += (((V_MAX*(1-k))/1.5)/((V_MAX*(k+1))/1.5));
            wb_motor_set_velocity(MTD, ((V_MAX*(k+1))/1.5));
            wb_motor_set_velocity(MTE, ((V_MAX*(1-k))/1.5)); 
            cont++;                   
        }       
    } else
    {
        if(lue < 1000)
        {
            wb_motor_set_velocity(MTD, 10.0);
            wb_motor_set_velocity(MTE, 9.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                lue = wb_distance_sensor_get_value(USE);
                if (lue < 670)
                {
                    break;
                }                      
            }
            wb_motor_set_velocity(MTD, 0.0);
            wb_motor_set_velocity(MTE, 0.0);
        } else
        {
            //vai para o lado indicado:
            wb_motor_set_velocity(MTD, 10.0);
            wb_motor_set_velocity(MTE, 5.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                lue = wb_distance_sensor_get_value(USE);
                luc = wb_distance_sensor_get_value(USC);
                if ((luc < 800) || (lue < 800))
                {
                    break;
                }             
            }
            //Alinhar
            wb_motor_set_velocity(MTD, 1.0);
            wb_motor_set_velocity(MTE, 10.0);
            while (wb_robot_step(TIME_STEP) != -1) 
            {
                lue = wb_distance_sensor_get_value(USE);
                if (lue < 800)
                {
                    break;
                }                      
            }
            wb_motor_set_velocity(MTD, 0.0);
            wb_motor_set_velocity(MTE, 0.0);
        }
        long long int cont=0;
        //aprende a circunferência do labirinto
        while (wb_robot_step(TIME_STEP) != -1) 
        {
            lue = wb_distance_sensor_get_value(USE);
            if(lue >= 1000)
            {
              if(skip == 0)
              {
                curve(200, 0.3, V_MAX);
                return 0;
              }else
              {
                wb_motor_set_velocity(MTD, V_MAX);
                wb_motor_set_velocity(MTE, (0.89*V_MAX*((float)c/cont)));
                delay(0.5);
                while (wb_robot_step(TIME_STEP) != -1) 
                {
                  lue = wb_distance_sensor_get_value(USE);
                  if(lue < 1000)
                  {
                    break;
                  }
                }
                skip--;
                continue;
              }
            }
            float k = ((900-lue)/100.0);
            c = (((V_MAX*(k+1))/1.5)/((V_MAX*(1-k))/1.5));
            som += c;
            printf("\n=> %.2f", c);
            wb_motor_set_velocity(MTD, ((V_MAX*(1-k))/1.5));
            wb_motor_set_velocity(MTE, ((V_MAX*(k+1))/1.5)); 
            cont++;                   
        }
    }    
}

int main() 
{
    wb_robot_init();

    //Definindo as tags dos dispositivos:
    MTD = wb_robot_get_device("roda_direita");
    MTE = wb_robot_get_device("roda_esquerda");  
    LFE = wb_robot_get_device("seguidor_de_linha1");
    LFCE = wb_robot_get_device("seguidor_de_linha2");
    LFC = wb_robot_get_device("seguidor_de_linha3");
    LFCD = wb_robot_get_device("seguidor_de_linha4");
    LFD = wb_robot_get_device("seguidor_de_linha5");
    ENC_D = wb_robot_get_device("encoder_direito");
    ENC_E = wb_robot_get_device("encoder_esquerdo");
    USE = wb_robot_get_device("ds_ultrassom1");
    USC = wb_robot_get_device("ds_ultrassom2");
    USD = wb_robot_get_device("ds_ultrassom3");

    //Inicializando os dispositivos:
    wb_motor_set_position(MTD, INFINITY);
    wb_motor_set_position(MTE, INFINITY);
    wb_motor_set_velocity(MTD, 0.0);
    wb_motor_set_velocity(MTE, 0.0);
    wb_distance_sensor_enable(LFE, TIME_STEP);
    wb_distance_sensor_enable(LFCE, TIME_STEP);
    wb_distance_sensor_enable(LFC, TIME_STEP);
    wb_distance_sensor_enable(LFCD, TIME_STEP);
    wb_distance_sensor_enable(LFD, TIME_STEP);
    wb_position_sensor_enable(ENC_D, TIME_STEP);
    wb_position_sensor_enable(ENC_E, TIME_STEP);
    wb_distance_sensor_enable(USE, TIME_STEP);
    wb_distance_sensor_enable(USC, TIME_STEP);
    wb_distance_sensor_enable(USD, TIME_STEP);  

    int mode = 1; //comeco pelo seguidor de linha

    //Laco de comandos principal:
    while (wb_robot_step(TIME_STEP) != -1) 
    {
        switch (mode)
        {
            case 1: //line follower
                LCD = wb_distance_sensor_get_value(LFCD);
                LCE = wb_distance_sensor_get_value(LFCE);
                LC = wb_distance_sensor_get_value(LFC);
                LD = wb_distance_sensor_get_value(LFD);
                LE = wb_distance_sensor_get_value(LFE);
                lue = wb_distance_sensor_get_value(USE);
                luc = wb_distance_sensor_get_value(USC);
                lud = wb_distance_sensor_get_value(USD);

                //Encerra o seguidor de linha
                if((lue < 1000) || (luc < 1000) || (lud < 1000))
                {
                   mode = 2;
                   break;
                }

                //printf("\nLD: %.2f", LD);
                //printf("\nLE: %.2f", LE);

                //Obedece a sinalizacao
                if ((LD < 110) && (LE > 115) && (wb_robot_get_time() > comptime))
                {
                    curve(-45, 0.1, V_MAX);
                    linecurve('D', 0.1, V_MAX);
                }
                if ((LE < 110) && (LD > 115) && (wb_robot_get_time() > comptime))
                {
                    curve(45, 0.1, V_MAX);
                    linecurve('E', 0.1, V_MAX);
                }   

                //Curva acentuada
                if ((LD > 130) && (LE < 125) && (wb_robot_get_time() > comptime))
                {
                    curve(-45, 0.07, V_MAX);
                    linecurve('D', 0.07, V_MAX);
                }
                if ((LE > 130) && (LD < 125) && (wb_robot_get_time() > comptime))
                {
                    curve(45, 0.07, V_MAX);
                    linecurve('E', 0.07, V_MAX);
                } 

                //Segue a linha
                if(fabs(LCD-LCE) < lim)
                {                
                  if(LCD > (LCE+tol))
                  {
                      float k = (1-((LCD-LCE)/20.0));
                      //printf("\n%.2f", k);
                      wb_motor_set_velocity(MTD, (V_MAX*k));
                      wb_motor_set_velocity(MTE, V_MAX);
                  }
                  if(LCD < (LCE+tol))
                  {
                      float k = (1-((LCE-LCD)/20.0));
                      //printf("\n%.2f", k);
                      wb_motor_set_velocity(MTD, V_MAX);
                      wb_motor_set_velocity(MTE, (V_MAX*k));
                  }
                 } else
                 {
                   if(LCD > (LCE+tol))
                    {
                      wb_motor_set_velocity(MTD, -V_MAX);
                      wb_motor_set_velocity(MTE, V_MAX);
                    }
                    if(LCD < (LCE+tol))
                    {
                      wb_motor_set_velocity(MTD, V_MAX);
                      wb_motor_set_velocity(MTE, -V_MAX);
                    }
                 }
                 
                 
                
            break;
            case 2: //labirinto
                nonclockwise_seek('E', 0);
                mode = 0;
            break;            
        }
        if(mode == 0)
        {
          break;
        }
    };
        
    wb_robot_cleanup();

    return 0;
}