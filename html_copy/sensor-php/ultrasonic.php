<?php
    $command = escapeshellcmd('python3 ultrasonic.py');
    $output = shell_exec($command);
    echo $output;
    echo "robot is moving forward";

?>
