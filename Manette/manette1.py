import pygame
import time

def print_controller_info():
    """Print detailed information about connected controllers"""
    pygame.init()
    pygame.joystick.init()
    
    print("Controller Diagnostic Tool")
    print("=========================")
    print(f"Number of controllers detected: {pygame.joystick.get_count()}")
    
    if pygame.joystick.get_count() == 0:
        print("No controllers found. Please connect a controller and try again.")
        return
    
    # Initialize the first controller found
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"\nController name: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")
    
    print("\nMOVE THE CONTROLLER STICKS AND TRIGGERS TO SEE THEIR VALUES")
    print("Press Ctrl+C to exit\n")
    
    try:
        while True:
            pygame.event.pump()
            
            # Print all axis values
            print("\033[H\033[J")  # Clear screen (works in most terminals)
            print("AXIS VALUES:")
            print("===========")
            for i in range(joystick.get_numaxes()):
                value = joystick.get_axis(i)
                # Only print if significant movement
                bar = "█" * int((value + 1) * 10)
                print(f"Axis {i}: {value:.2f} {bar}")
            
            # Print all button values
            print("\nBUTTON VALUES:")
            print("=============")
            for i in range(joystick.get_numbuttons()):
                value = joystick.get_button(i)
                status = "PRESSED" if value else "released"
                if value:
                    print(f"Button {i}: {status}")
            
            # Print all hat values
            print("\nHAT VALUES:")
            print("==========")
            for i in range(joystick.get_numhats()):
                value = joystick.get_hat(i)
                print(f"Hat {i}: {value}")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nExiting controller diagnostic tool")
    
    finally:
        pygame.quit()

if __name__ == "__main__":
    print_controller_info()