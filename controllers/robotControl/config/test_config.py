from robot_config import OPERATION_MODE, CLEANING_AREAS

print(f"OPERATION_MODE: {OPERATION_MODE}")
print(f"Default area: {OPERATION_MODE['default_area']}")
print(f"Boundary points: {CLEANING_AREAS[OPERATION_MODE['default_area']]}") 