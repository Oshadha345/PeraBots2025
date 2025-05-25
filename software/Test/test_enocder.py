import sys
import os

# Ensure the parent directory is in the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from sensors import MockEncoder

encoder = MockEncoder()

for _ in range(5):
    ticks = encoder.get_ticks()
    distance = encoder.get_distance()
    print(f"Ticks: {ticks}, Distance: {distance:.2f} m")
