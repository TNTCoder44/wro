###
### ScannerSubsystem for detecting colors of the samples
###

class Colors:
    NONE=0 
    WHITE=1
    BLACK=2
    GREEN=3
    BLUE=4
    RED=5
    YELLOW=6
    

class ScannerSubsystem:
    def __init__(self, color_sensor):
        self.color_sensor = color_sensor

    # scan for colors using hsv values for more accurate sample detection
    def scan(self, iterations: int = 25):
        hsv_total = [0, 0, 0]

        for _ in range(iterations):
            hsv = self.color_sensor.hsv()
            for i in range(3):
                hsv_total[i] += hsv[i]

        avg = [hsv_total[i] / iterations for i in range(3)]
 
        if avg[2] < 15:
            return Colors.NONE 

        # possible colors for samples:
        #   white, red, green, yellow
        if avg[1] < 10: 
            return Colors.WHITE
        
        if avg[0] > 320 or avg[0] < 40:
            return Colors.RED
        
        elif avg[0] < 70 and avg[0] > 50:
            return Colors.YELLOW
        
        elif avg[0] < 130 and avg[0] > 110:
            return Colors.GREEN
        
        return Colors.NONE
        
