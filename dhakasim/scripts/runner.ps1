# Set the path to your Java executable
$javaExecutable = 'C:\Program Files\Eclipse Adoptium\jdk-21.0.5.11-hotspot\bin\java.exe'

# Set the Java program and its arguments
$javaProgram = 'thesisfinal.DhakaSim'
$javaProgramArgumentsTemplate = '@C:\Users\User\AppData\Local\Temp\cp_cu8zl7b9otg1saurzn4yt8bag.argfile'

# Initialize variables
$numIterations = 10

$averageSpeedPattern = "speed: ([\d.]+)"
$averageWaitingTimePattern = "waiting time: ([\d.]+)"

# Initialize result arrays
$results = @()
$resultsFile = 'results.txt'

        # $javaProgramArguments = $javaProgramArgumentsTemplate + " $arg1 $arg2"

        # Initialize variables for each iteration
        $averageSpeed = 0.0
        $averageWaitingTime = 0.0
        
        # Run the Java program 10 times
        for ($i = 1; $i -le $numIterations; $i++) {
            # Write-Host "Running iteration $i for arguments ($arg1, $arg2)..."
            
            # Run the Java program and capture the output
            & $javaExecutable $javaProgramArgumentsTemplate $javaProgram > log.txt
            # & $javaExecutable $javaProgramArgumentsTemplate $javaProgram >> log2.txt
            $logContent = Get-Content -Path 'log.txt'

            foreach ($line in $logContent) {
                # Check if the line contains average speed information
                if ($line -match $averageSpeedPattern) {
                    $speed = [double]::Parse($matches[1])
                }
                elseif ($line -match $averageWaitingTimePattern) {
                    $waitingTime = [double]::Parse($matches[1])
                }                
                
            }

            Write-Host "Average Speed: $speed"
            Write-Host "Average Waiting Time: $waitingTime"
            
            # Write the speed and waiting time to the results file
            $resultString = "$speed,$waitingTime,$motorizedSpeed,$nonMotorizedSpeed,$motorizedWaitingTime,$nonMotorizedWaitingTime"
            # $resultString >> results.txt
            $resultString | Out-File -FilePath $resultsFile -Append

            $averageSpeed += $speed
            $averageWaitingTime += $waitingTime            

        }

        # Calculate the average result for the current set of arguments
        $averageSpeed = $averageSpeed / $numIterations
        $averageWaitingTime = $averageWaitingTime / $numIterations        


        # Store the results
        $results += [PSCustomObject]@{            
            AverageSpeed = $averageSpeed
            AverageWaitingTime = $averageWaitingTime
           
        }

# Display the results
$results | Format-Table -AutoSize

# Convert results to a string and append to the results.txt file
$results | Out-String | Out-File -FilePath $resultsFile -Append
