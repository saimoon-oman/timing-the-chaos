# Set the path to your Java executable
$javaExecutable = 'C:\Program Files\Eclipse Adoptium\jdk-21.0.5.11-hotspot\bin\java.exe'

# Set the Java program and its arguments
$javaProgram = 'thesisfinal.DhakaSim'
$javaProgramArgumentsTemplate = '@C:\Users\User\AppData\Local\Temp\cp_cu8zl7b9otg1saurzn4yt8bag.argfile'

# Initialize variables
$numIterations = 15

$spcar_Pattern = "sp_car: ([\d.]+)"
$spbike_Pattern = "sp_bike: ([\d.]+)"
$pscar_Pattern = "ps_car: ([\d.]+)"
$psbike_Pattern = "ps_bike: ([\d.]+)"


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
            $logContent = Get-Content -Path 'log.txt'

            foreach ($line in $logContent) {
                # Check if the line contains average speed information
                if ($line -match $spcar_Pattern) {
                    $spcar = [double]::Parse($matches[1])
                }
                elseif ($line -match $spbike_Pattern) {
                    $spbike = [double]::Parse($matches[1])
                }
                elseif ($line -match $pscar_Pattern) {
                    $pscar = [double]::Parse($matches[1])
                }
                elseif ($line -match $psbike_Pattern) {
                    $psbike = [double]::Parse($matches[1])
                }
            }

            # Write-Host "Average Speed: $speed"
            # Write-Host "Average Waiting Time: $waitingTime"
            Write-Host "done"
            
            $resultString = "$spcar,$spbike,$pscar,$psbike"
            # $resultString >> results.txt
            $resultString | Out-File -FilePath $resultsFile -Append
            
            $average_spcar += $spcar
            $average_spbike += $spbike
            $average_pscar += $pscar
            $average_psbike += $psbike           
           

        }

        # Calculate the average result for the current set of arguments
        $average_spcar = $average_spcar / $numIterations
        $average_spbike = $average_spbike / $numIterations
        $average_pscar = $average_pscar / $numIterations
        $average_psbike = $average_psbike / $numIterations

        # Store the results
        $results += [PSCustomObject]@{            
            Average_sp_car = $average_spcar
            Average_sp_bike = $average_spbike
            Average_ps_car = $average_pscar
            Average_ps_bike = $average_psbike            
        }

# Display the results
$results | Format-Table -AutoSize

# Convert results to a string and append to the results.txt file
$results | Out-String | Out-File -FilePath $resultsFile -Append
