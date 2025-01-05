import re
import os
import sys

# List of files to excuse (constants and things we didnt make and stuff we wont use)
excused_files = ["GlobalConstants.java", "Drive.java", "TunerConstants.java", "BuildConstants.java", "LocalADStarAK.java", "VisionUtil.java", "SwerveModule.java", "VisionIOSim.java", "ElevatorConstants.java", "IntakeConstants.java"]

# Not really dirs becasue the full ones didnt work
excused_dirs = [
    "bin",
    "build",
    "src/main/java/frc/robot/pioneersLib"
]

# Weird stuff that shouldn't go in constants, dont put function/var names in here theyre already checked
excused_cases = ["ModuleIOSparkMax", "case", "new Module(", "new BaseStatusSignal[", "BaseStatusSignal.waitForAll(", "new ModuleIOHybrid(", "Math.pow(", "+=", "drive.getRotation()", "autoChooser.addOption(", "static final", "getRealTimestamp", "antiJitterThreshold", "trackWidth", "i < 4", "SwerveModuleState[4]", "gearRatio", "Math.PI", "private final", "/ 360", "/360", "* 360", "*360", "DCMotor.get", "/60", "/ 60"]

def check_for_magic_numbers(file_path):
    magic_numbers = []
    
    # Number pattern makes sure number isnt in a var and detects all numbers that arent in a function/var
    number_pattern = r'(?<!\w)-?(?:\d*\.\d+|\d+\.?)\b(?!\w)'
    zero_pattern = r'^0*\.?0+$'

    with open(file_path, 'r') as file:
        lines = file.readlines()
        in_comment_block = False
        
        for line_number, line in enumerate(lines, 1):
            line = line.strip()

            # Skip empty lines
            if not line:
                continue
            
            # Skip big comments
            if '/*' in line:
                in_comment_block = True
            # If someone like put a thing after a multi-line comment end this wouldnt work but idk if you can even do that
            if '*/' in line:
                in_comment_block = False
                continue
            if in_comment_block:
                continue

            # Skip regular comments and imports
            if line.startswith('//') or line.startswith('import'):
                continue
            
            # Skips excused cases
            if any(case in line for case in excused_cases):
                continue

            numbers = re.findall(number_pattern, line)
            for number in numbers:
                # Skip for zeroes, they're fine as is
                if re.match(zero_pattern, number):
                    continue                
                magic_numbers.append((number, line_number))
    return magic_numbers

def is_file_excused(file_path, project_root):
    filename = os.path.basename(file_path)
    if filename in excused_files:
        return True

    relative_path = os.path.relpath(file_path, project_root)
    for excluded_dir in excused_dirs:
        if relative_path.startswith(excluded_dir) or excluded_dir in relative_path:
            return True

    return False

def scan_directory(directory):
    magic_number_count = 0
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.java'):
                file_path = os.path.join(root, file)
                if not is_file_excused(file_path, directory):
                    magic_numbers = check_for_magic_numbers(file_path)
                    if magic_numbers:
                        print(f"In {file_path}:")
                        for number, line_number in magic_numbers:
                            print(f"  Line {line_number}: Magic number {number}")
                            magic_number_count += 1
    return magic_number_count

if __name__ == "__main__":
    # Get scripts dir
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Move up 2 dirs to get the project root
    project_root = os.path.dirname(os.path.dirname(script_dir))
    
    print(f"Scanning directory: {project_root}")
    total_magic_numbers = scan_directory(project_root)

    # Fails if magic numbers are in any non excused locations    
    if total_magic_numbers > 0:
        print(f"\nTotal magic numbers found: {total_magic_numbers}.\nPlease put these in  Constanats.java!")
        sys.exit(1)
    else:
        print("\nNo Magic Number Found")
        sys.exit(0)