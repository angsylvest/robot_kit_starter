<?php
    $command = escapeshellcmd('python3 motor_exe.py');
    $output = shell_exec($command); 
    echo $output;   
    echo "robot is moving forward"; 

?>
