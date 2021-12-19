import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math as m
import pandas as pd

# Config camera
pipe = rs.pipeline()
config = rs.config()
config.enable_device_from_file("pruebaRoll.bag") #archivo .bag al que se le aplicará el filtro, OMITIR si se tiene la camara conectadad
config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)



# Depth stream @ 848x480 px, 1 canal de 16 bits, 30 fps
config.enable_stream(rs.stream.depth)
# Color stream @ 1280x720 px, 3 canales de 8 bits, 30 fps
config.enable_stream(rs.stream.color) 
# Enable bag saving
#config.enable_record_to_file('test.bag') #NO se puede desde un bag mismo

pipe.start(config)
# Video handlers. 1 for RGB, 1 for depth
fourcc =  cv2.VideoWriter_fourcc(*'XVID')
out_rgb = cv2.VideoWriter('color.avi', fourcc, 30.0, (1280,720))
out_d = cv2.VideoWriter('depth.avi', fourcc, 30.0, (848,480))
out_filtro = cv2.VideoWriter('filtro.avi', fourcc, 30.0, (1280,720))

def motion2vec3(motion_data):
    return np.asarray([motion_data.x, motion_data.y, motion_data.z])

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

# Matrices de rotacion

try:
    #valores iniciales
    t_0=0
    angulox=0
    anguloy=0
    anguloz=0
    Roll_0=0
    l_t=[]
    l_roll=[]
    l_rollAc=[]
    l_rollGy=[]
    l_pitch=[]
    l_pitchAc=[]
    l_pitchGy=[]
    l_yaw=[]
    l_yawAc=[]
    l_yawGy=[]
    l_ac0=[]
    l_ac1=[]
    l_ac2=[]
    l_gy0=[]
    l_gy1=[]
    l_gy2=[]
    while True:
        f = pipe.wait_for_frames()
        # #IMU processing not working. Returns null pointer exception
        # accel_data = motion2vec3(frames[0].as_motion_frame().get_motion_data())
        # gyro_data = motion2vec3(frames[1].as_motion_frame().get_motion_data())
        # print(f"Accel: {accel_data}")
        # print(f"Gyro: {gyro_data}")
        
        # ------------CODIGO PARA LOS ANGULOS---------
        accel = accel_data(f[3].as_motion_frame().get_motion_data())
        gyro = gyro_data(f[2].as_motion_frame().get_motion_data())
        t = f.get_timestamp()
        if t_0==0:
            dt=0
        else:
            dt= (t-t_0) /1000
            if dt<0:
                dt=0
        t_0 = t
        
        R= np.sqrt(accel[0]**2+accel[1]**2+accel[2]**2)
        #Las rotaciones en ángulos euler (desde https://github.com/GruffyPuffy/imutest/blob/master/imutest.cpp)

        # se calculan las rotaciones integrando la velocidad angular (sin más correcciones, no deberia ser tan preciso)
       # print('anguloz: ',anguloz)
        Roll_gy = angulox + dt*gyro[2]
        Pitch_gy = anguloy + dt*gyro[0]
        Yaw_gy = anguloz + dt*gyro[1]
        print('roll_gy: ',Roll_gy, 'dt: ', dt, 'gyro[2]: ', gyro[2])
        #valores estimados en reposo 
        
        if R == 0: #para que no se indetermine
            Roll=Roll_gy
            Pitch=Pitch_gy
            Yaw=Yaw_gy
        else:
            Roll_ac = np.arccos(accel[0] / R)
            Pitch_ac = np.arccos(accel[2] / R)
            Yaw_ac = np.arccos(accel[1] / R)
        
        print(angulox,anguloy,anguloz)    
        if angulox == 0:
            angulox=Roll_ac
            anguloy=Pitch_ac
            anguloz=Yaw_ac
        print('despues: ',angulox,anguloy,anguloz)   
        Roll_gy = angulox + dt*gyro[2]
        Pitch_gy = anguloy + dt*gyro[0]
        Yaw_gy = anguloz + dt*gyro[1]
        print ('roll:gy despues del if: ', Roll_gy)
        # se incorporan ambos valores segun el Filtro Complementario:
        alph= 0.98 # ponderación del valor del giroscopio en relacion al del acelerometro (arbitrario)
        Roll= alph*Roll_gy+(1-alph)*Roll_ac
        Pitch= alph*Pitch_gy+(1-alph)*Pitch_ac
        Yaw= Yaw_gy #despreciamos yaw_ac ya que se alejan mucho los valores
        print ('roll: ', Roll_gy, 'pitch:', Pitch_gy, 'yaw: ', Yaw_gy)

        angulox= Roll_gy
        anguloy= Pitch_gy
        anguloz= Yaw_gy
        
        # listas con los datos
        t_i= 1638911088.59946+741.546611309051 #obtenido experimentalmente para el grafico
        l_t.append((t/1000)-t_i)
        l_roll.append(np.degrees(Roll))
        l_rollAc.append(np.degrees(Roll_ac))
        l_rollGy.append(np.degrees(Roll_gy))
        l_pitch.append(np.degrees(Pitch))
        l_pitchAc.append(np.degrees(Pitch_ac))
        l_pitchGy.append(np.degrees(Pitch_gy))
        l_yaw.append(np.degrees(Yaw))
        l_yawAc.append(np.degrees(Yaw_ac))
        l_yawGy.append(np.degrees(Yaw_gy))
        l_ac0.append(accel[0])
        l_ac1.append(accel[1])
        l_ac2.append(accel[2])
        l_gy0.append(gyro[0])
        l_gy1.append(gyro[1])
        l_gy2.append(gyro[2])
        
    
        
         
        #datos = pd.DataFrame([[l_t], [l_rollAc], [l_pitchAc], [l_yawAc]], columns=['Tiempo (ms)', 'Roll (angulo en eje x) acelerometro (rad)', 'Pitch (angulo en eje y) acelerometro (rad)', 'Pitch (angulo en eje z) acelerometro (rad)'])
        datos = pd.DataFrame({'Tiempo (s)': l_t, 'Roll (angulo en eje x) acelerometro (°)':l_rollAc,'Pitch (angulo en eje y) acelerometro (°)':l_pitchAc, 'Yaw (angulo en eje z) acelerometro (°)': l_yawAc, 'Roll giroscopio (°)': l_rollGy, 'Pitch giroscopio (°)': l_pitchGy, 'Yaw giroscopio (°)': l_yawGy, 'Roll filtro Complementario (°)': l_roll, 'Pitch filtro Complementario (°)': l_pitch, 'Yaw filtro Complementario (°)': l_yaw})  
        datosOg= pd.DataFrame({'Tiempo (s)': l_t,'acelerometro [0]': l_ac0, 'aclerometro [1]': l_ac1,'acelerometro [2]': l_ac2, 'giroscopio [0]': l_gy0, 'giroscopio [1]': l_gy1, 'giroscopio [2]': l_gy2})  
        datos.to_csv('datos.csv', index=False)
        datosOg.to_csv('datosOg.csv', index=False)
         
        
        
        #--------------CODIGO PARA GUARDAR IMAGENES------
        depth_frame = f.get_depth_frame()
        rgb_frame = f.get_color_frame()
        if not depth_frame or not rgb_frame:
            print("Didn't get images")
            continue
        # Transform data to numpy arrays
        depth_img = np.asanyarray(depth_frame.get_data())
        color_img = np.asanyarray(rgb_frame.get_data())
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        # Convert depth image to colors with Jet colormap. Check alpha value
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
        # Store images in videos
        out_rgb.write(color_img)
        out_d.write(depth_colormap)
        #show_img = np.hstack((color_img, depth_colormap))
        cv2.namedWindow('RS', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RS', color_img)
        
        #   Rotacion de la imagen
        M=cv2.getRotationMatrix2D((0,0),m.degrees(Roll)-m.degrees(Roll_0),1)
        print('diferencia en grados: ',m.degrees(Roll)-m.degrees(Roll_0))
        filtro_img= cv2.warpAffine(color_img,M,(1280,720))
        
        show_img = np.hstack((color_img, filtro_img))
        out_filtro.write(filtro_img)
        cv2.imshow('RS', filtro_img)
        Roll_0=Roll
        
        
        
        # Exit on pressing q key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Close RS pipe, video handlers and windows
    pipe.stop()
    out_d.release()
    out_rgb.release()
    cv2.destroyAllWindows()
    
    
