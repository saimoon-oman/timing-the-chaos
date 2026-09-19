# Set the path to your Java executable
$javaExecutable = 'C:\Program Files\Eclipse Adoptium\jdk-21.0.5.11-hotspot\bin\java.exe'

# Set the Java program and its arguments
$javaProgram = 'thesisfinal.DhakaSim'
$javaProgramArgumentsTemplate = '@C:\Users\User\AppData\Local\Temp\cp_cu8zl7b9otg1saurzn4yt8bag.argfile'

# Initialize variables
$numIterations = 10

$averageSpeedPattern = "speed: ([\d.]+)"
$averageWaitingTimePattern = "waiting time: ([\d.]+)"
$averageMotorizedSpeedPattern = "motorized speed: ([\d.]+)"
$averageNonMotorizedSpeedPattern = "non-motorized speed: ([\d.]+)"
$averageMotorizedWaitingTimePattern = "motorized waiting time: ([\d.]+)"
$averageNonMotorizedWaitingTimePattern = "non-motorized waiting time: ([\d.]+)"


# Initialize result arrays
$results = @()
$resultsFile = 'results.txt'

        # $javaProgramArguments = $javaProgramArgumentsTemplate + " $arg1 $arg2"

        # Initialize variables for each iteration
        $averageSpeed = 0.0
        $averageWaitingTime = 0.0
        $averageMotorizedspeed = 0.0
        $averageNonMotorizedSpeed = 0.0
        $averageMotorizedWaitingTime = 0.0
        $averageNonMotorizedWaitingTime = 0.0

        # Run the Java program 10 times
        for ($i = 1; $i -le $numIterations; $i++) {
            # Write-Host "Running iteration $i for arguments ($arg1, $arg2)..."
            
            # Run the Java program and capture the output
            & $javaExecutable $javaProgramArgumentsTemplate $javaProgram > log.txt
            # & $javaExecutable $javaProgramArgumentsTemplate $javaProgram >> log2.txt
            $logContent = Get-Content -Path 'log.txt'

            foreach ($line in $logContent) {
                # Check if the line contains average speed information
                
                # Check if the line contains average waiting time information
                if ($line -match $averageNonMotorizedSpeedPattern) {
                    $nonMotorizedSpeed = [double]::Parse($matches[1])
                }
                elseif ($line -match $averageNonMotorizedWaitingTimePattern) {
                    $nonMotorizedWaitingTime = [double]::Parse($matches[1])
                }
                elseif($line -match $averageMotorizedSpeedPattern) {
                    $motorizedSpeed = [double]::Parse($matches[1])
                }
                elseif ($line -match $averageMotorizedWaitingTimePattern) {
                    $motorizedWaitingTime = [double]::Parse($matches[1])
                }                
                elseif ($line -match $averageSpeedPattern) {
                    $speed = [double]::Parse($matches[1])
                }
                elseif ($line -match $averageWaitingTimePattern) {
                    $waitingTime = [double]::Parse($matches[1])
                } 
            }

            Write-Host "Average Speed: $speed"
            Write-Host "Average Waiting Time: $waitingTime"
            Write-Host "Average Motorized Speed: $motorizedSpeed"
            Write-Host "Average Non-Motorized Speed: $nonMotorizedSpeed"
            Write-Host "Average Motorized Waiting Time: $motorizedWaitingTime"
            Write-Host "Average Non-Motorized Waiting Time: $nonMotorizedWaitingTime"
            
            # Write the speed and waiting time to the results file
            $resultString = "$speed,$waitingTime,$motorizedSpeed,$nonMotorizedSpeed,$motorizedWaitingTime,$nonMotorizedWaitingTime"
            # $resultString >> results.txt
            $resultString | Out-File -FilePath $resultsFile -Append

            $averageSpeed += $speed
            $averageWaitingTime += $waitingTime
            $averageMotorizedspeed += $motorizedSpeed
            $averageNonMotorizedSpeed += $nonMotorizedSpeed
            $averageMotorizedWaitingTime += $motorizedWaitingTime
            $averageNonMotorizedWaitingTime += $nonMotorizedWaitingTime                        
           

        }

        # Calculate the average result for the current set of arguments
        $averageSpeed = $averageSpeed / $numIterations
        $averageWaitingTime = $averageWaitingTime / $numIterations
        $averageMotorizedspeed = $averageMotorizedspeed / $numIterations
        $averageNonMotorizedSpeed = $averageNonMotorizedSpeed / $numIterations
        $averageMotorizedWaitingTime = $averageMotorizedWaitingTime / $numIterations
        $averageNonMotorizedWaitingTime = $averageNonMotorizedWaitingTime / $numIterations


        # Store the results
        $results += [PSCustomObject]@{            
            AverageSpeed = $averageSpeed
            AverageWaitingTime = $averageWaitingTime
            AverageMotorizedSpeed = $averageMotorizedspeed
            AverageMotorizedWaitingTime = $averageMotorizedWaitingTime
            AverageNonMotorizedSpeed = $averageNonMotorizedSpeed
            AverageNonMotorizedWaitingTime = $averageNonMotorizedWaitingTime
        }

# Display the results
$results | Format-Table -AutoSize

# Convert results to a string and append to the results.txt file
$results | Out-String | Out-File -FilePath $resultsFile -Append
