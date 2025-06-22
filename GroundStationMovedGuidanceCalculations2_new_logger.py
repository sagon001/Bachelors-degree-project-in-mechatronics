import asyncio
from mavsdk import System
import math
import os
import time
import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
#start C:\Users\samue\OneDrive\Skrivbord\Programmering\mavsdk_server.exe serial:///COM3 -p 50051

##### logger #################
import logging

# Create named logger
logger = logging.getLogger("my_logger")
logger.setLevel(logging.INFO)  # Controls what gets through

# Prevent log from bubbling up to root logger
logger.propagate = False

# Clear existing handlers (in case this gets run multiple times)
if logger.hasHandlers():
    logger.handlers.clear()

# Create file handler
file_handler = logging.FileHandler("logger.log")
file_handler.setLevel(logging.INFO)  # Make sure it matches or is lower than logger level

# Optional: clean formatting
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)

# Add handler to logger
logger.addHandler(file_handler)

############



class DroneState:
    """ Shared state object to hold telemetry data """
    def __init__(self):
        self.pressure = 0
        self.heading = 0
        self.velocity = None
        self.V0x=0
        self.V0y=0
        self.position = None
        self.altitude = 0
        self.distance = 100
        self.imageWidth=0
        self.imageHeight=0
        self.dx=2000
        self.dy=2000
        self.servoAvailable = True
        self.updatedDistance=False
    def updatePressure(self, value):
        self.pressure = value
    def updateHeading(self, value):
        self.heading = value
    def updateVelocity(self, value):
        self.velocity = value
    def updatePosition(self,value):
        self.position=value
        #print(self.position)
    def updateAltitude(self,value):
        self.altitude=value
    def updateDistance(self,value):
        self.distance=value #det här är uppfattade räckvidden till Apriltag
        self.updatedDistance=True
    def updateImage(self,width,height):
        self.imageWidth=width
        self.imageHeight=height
    def updateOffsets(self,dx,dy):
        self.dx=dx
        self.dy=dy
    def getState(self):
        #print(self.position)
        return {"heading": self.heading, "velocity": self.velocity, "position":self.position}
    def getRange(self):

        h=self.altitude
        g = 9.81
        if self.velocity != None:
            V0z=self.velocity.down_m_s  #(north_m_s, east_m_s, down_m_s))
            HeadingRads=math.radians(self.heading)
            #V0x=self.velocity.north_m_s
            self.V0x = self.velocity.north_m_s*math.cos(HeadingRads) + self.velocity.east_m_s*math.sin(HeadingRads)
            self.V0y = -self.velocity.north_m_s*math.sin(HeadingRads) +self.velocity.east_m_s*math.cos(HeadingRads)
            try:
                t = (V0z + math.sqrt(V0z**2 + 2 * g * h)) / g          #t=math.sqrt((2*height)/g) #definerat neråt som positivt
                                                                        #distance=abs(self.velocity)*time_to_ground
                r = self.V0x * t
                #print(f"Estimated Forward Distance Before Hitting Ground: {r:.2f} meters")
                return r
            except ValueError:
                print("Invalid time calculation")
                return 0
        else:
            print("velocity is None object, no range")
            return 0      

    def getRangeWithDrag(self):
        # Constants
        g = 9.81  # m/s^2
        rho = 1.2  # kg/m^3 (air density)
        C = 0.5  # Drag coefficient (sphere)
        radius = 0.0366  # meters 
        A = math.pi * radius**2  # cross-sectional area
        m = 0.145  # kg (mass of projectile)
        D = 0.5 * C * rho * A  # drag factor
        delta_t = 0.001  # time step (s)

        if self.velocity is None:
            print("velocity is None object, no range")
            return 0

        # Initial conditions
        h = self.altitude  # starting height (meters)
        HeadingRads = math.radians(self.heading)
        # Resolve velocity into world-frame X (forward) and Y (lateral) directions
        Vn = self.velocity.north_m_s
        Ve = self.velocity.east_m_s
        Vd = self.velocity.down_m_s  # positive down

        vx = Vn * math.cos(HeadingRads) + Ve * math.sin(HeadingRads)
        vy = -Vn * math.sin(HeadingRads) + Ve * math.cos(HeadingRads)
        vz = Vd  # downward

        x = 0  # horizontal distance
        y = h  # vertical height
        t = 0

        # Loop until the object hits the ground
        while y > 0:
            v = math.sqrt(vx**2 + vy**2 + vz**2)
            ax = -(D/m) * v * vx
            ay = -(D/m) * v * vy
            az = g - (D/m) * v * vz  # downward is positive

            # Euler integration
            vx += ax * delta_t
            vy += ay * delta_t
            vz += az * delta_t

            x += vx * delta_t
            y -= vz * delta_t  # subtract because vz is down
            t += delta_t

        return x  # This is the forward distance (range)     
    def getDistance(self):
        h=self.altitude
        d=self.distance
        d0x=d
        if d**2-h**2 < 0:
                print("No bueno, height is greater than detected distance")
        else:
            d0x=math.sqrt(d**2-h**2)
        return(d0x,self.updatedDistance)
async def run():
    # Init the drone
    #start C:\Users\samue\OneDrive\Skrivbord\Programmering\mavsdk_server.exe serial:///COM3 -p 50051
    state = DroneState()  # Create shared state object
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()
    #await drone.telemetry.set_rate_velocity_ned(20)
    await drone.telemetry.set_rate_position(10)
    #print("Drone sucesslfully connected")
    #print("mode: Direct delivery")
    print("What mode do you wish to enter? Enter ""1"" for Ballistic delivery or 2 for Direct delivery" )
    x=int(input())
    await drone.action.set_actuator(6,1000)

    # Start telemetry tasks
    asyncio.create_task(printPressure(drone,state))
    asyncio.create_task(printHeading(drone,state))
    asyncio.create_task(printVelocity(drone,state))
    asyncio.create_task(printPosition(drone,state))
    asyncio.create_task(printHeight(drone,state))
    if x==1:
        task1 = asyncio.create_task(BallisticDelivery(drone,state))  # Function for ballistic delivery
        task2 = asyncio.create_task(getDistance(state))
        #asyncio.create_task(getDistance(state))
        await asyncio.gather(task1,task2)   
    elif x==2:
        task1 = asyncio.create_task(DirectDelivery(drone, state))
        task2 = asyncio.create_task(getDistance(state))
        await asyncio.gather(task1,task2)
    else:
        print("Non acceptable input")
        return
    while True:
        await asyncio.sleep(1)
async def getDistance(state):
    # Load AprilTag dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary)
    # Define real-world tag size (meters)
    KNOWN_WIDTH = 0.34  # Adjust based on tag size
    FOCAL_LENGTH = 560  # Camera focal length
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            # Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Detect AprilTags
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i, corner in enumerate(corners):
                    # Get bounding box width in pixels
                    tag_width_pixels = np.linalg.norm(corner[0][0] - corner[0][1])
                    # Estimate distance
                    distance = (KNOWN_WIDTH * FOCAL_LENGTH) / tag_width_pixels
                    state.updateDistance(distance)
                    # Display ID and distance
                    center = tuple(np.mean(corner[0], axis=0).astype(int))
                    cv2.putText(frame, f"ID: {ids[i][0]}", (center[0] - 20, center[1] - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(frame, f"Distance: {distance:.2f}m", (center[0] - 20, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show frame
            cv2.imshow("AprilTag Distance Estimation", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            await asyncio.sleep(0.01)
            
    finally:
        cap.release()
        cv2.destroyAllWindows()
async def BallisticDelivery(drone,state):
    """ Periodically use the stored sensor data """
    while True:
        os.system("cls" if os.name == "nt" else "clear")  
        #current_state = state.getState()
        #print(f"Using Data -> Heading: {current_state['heading']}, Velocity:{current_state['velocity']},Position:{current_state['position']}")
        r=state.getRange()
        targetDistance,updatedDistance=state.getDistance()
        lower_bound = r * 0.9  # 10% below r
        upper_bound = r * 1.1  # 10% above r
        if lower_bound <= targetDistance <= upper_bound and updatedDistance==True:
            print("=====In range=====")
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            await drone.action.set_actuator(6,1400)
            try:
                logger.info(f"[{timestamp}] Ballistic delivery, FC altitude: {state.altitude}, Distance to apriltag: {state.distance}, V0x:{state.V0x},V0y:{state.V0y}\n")
                #with open("log.txt", "a") as f:
                #    f.write(f"[{timestamp}] Ballistic delivery, FC altitude: {state.altitude}, Distance to apriltag: {state.distance}, V0x:{state.V0x},V0y:{state.V0y}\n")#velocity.north_m_s
            except IOError as e:
                print(f"Failed to write to log file: {e}") 
            
            
            time.sleep(0.5)
            #print("Setting actuator 7 to 1100") 
            await drone.action.set_actuator(6,1000)
             
        else:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"Heading: {state.heading} Distance To Apriltag: {targetDistance:.2f} FC alt: {state.altitude} Aproximate range: ({lower_bound:.2f}-{upper_bound:.2f}) V0x:{state.V0x},V0y:{state.V0y}")
            pass
            if state.velocity is not None:
                try:
                    with open("speedLogV0x_västerut.txt", "a") as f:
                        f.write(f"[{timestamp}] North:{state.V0x}, East:{state.V0y}\n")
                except IOError as e:
                    print(f"Failed to write to log file: {e}")
            else:
                pass  
        await asyncio.sleep(0.1)  # Adjust delay as needed
async def DirectDelivery(drone,state):
        #Define the AprilTag dictionary (AprilTag 36h11 is commonly used)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    aruco_params = aruco.DetectorParameters()

    # Start video capture
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        print(state.velocity)

        ret, frame = cap.read()
        if not ret:
            break

        # Get frame dimensions
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2

        #Convert frame to grayscale for better processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect AprilTags
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                # Get the corner points of the detected tag
                corner_points = corners[i][0]  # Extract first marker points

                # Calculate the center of the detected AprilTag
                tag_center_x = int((corner_points[0][0] + corner_points[2][0]) / 2)
                tag_center_y = int((corner_points[0][1] + corner_points[2][1]) / 2)
                # Calculate the pixel distance from the image center
                dx = tag_center_x - center_x
                dy = tag_center_y - center_y
                state.updateOffsets(dx,dy)
                state.updateImage(width,height)
                # Draw the marker outline and its center
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)
                
                # Display the distance
                text = f"dx: {dx}px, dy: {dy}px"
                cv2.putText(frame, text, (tag_center_x + 10, tag_center_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            os.system("cls" if os.name == "nt" else "clear")  
            current_state = state.getState()
            print("Flightcontroller altitude:" + str(state.altitude) +" Vistance to apriltag:" + str(state.distance)+"Velocity"+str(state.velocity))
            #print(f"Using Data -> Heading: {current_state['heading']}, Velocity:{current_state['velocity']},Position:{current_state['position']}")
            # Check if tag is centered (within 5% of frame center)
            if abs(state.dx) <= state.imageWidth * 0.05 and abs(state.dy) <= state.imageHeight * 0.05:
                #ATTENTION VI KANSKEBORDE LÄGGA IN HANTERING SÅ ATT HASTIGHETERNA I X OCH Y LED SETT UPPIFRÅN ÄR NOLL NÄR VI SLÄPPER
                #print("Centered AprilTag detected in center!")
                print("Starting actuator movement")
                await drone.action.set_actuator(6, 1600)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                logstring = f"[{timestamp}] Direct delivery, Relative altitude: {state.altitude},  Aprox distance to apriltag: {state.distance}, Velocity:{state.velocity}, Heading: {state.heading}"
                try:
                    #with open("logDistanceEstimation.txt", "a") as f:
                    #    f.write(logstring)
                    logger.info(logstring)
                    #exit()
                except IOError as e:
                    print(f"Failed to write to log file: {e}")
                print("Actuator movement completed")
                await asyncio.sleep(0.5)
                #await drone.action.set_actuator(6, 1000)

            else:
                pass
        #Draw a cross at the image center
        cv2.drawMarker(frame, (center_x, center_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        # Show the output
        cv2.imshow('AprilTag Tracking', frame)
        await asyncio.sleep(0.01)  # Adjust as needed
        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()               
async def printPressure(drone, state):
    async for scaled_pressure in drone.telemetry.scaled_pressure():
        state.updatePressure(scaled_pressure.absolute_pressure_hpa)
async def printHeading(drone, state):
    async for heading in drone.telemetry.heading():
        state.updateHeading(heading.heading_deg)
async def printVelocity(drone, state):
    async for speed_NED in drone.telemetry.velocity_ned():
        state.updateVelocity(speed_NED) #state.update_velocity(speed_NED.north_m_s)
async def printPosition(drone,state):
    async for position in drone.telemetry.position():
        #print(position)
        state.updatePosition(position)     
async def printHeight(drone,state):
    async for position in drone.telemetry.position():
            #print(position)
            state.updateAltitude(position.relative_altitude_m)  
if __name__ == "__main__":
    asyncio.run(run())