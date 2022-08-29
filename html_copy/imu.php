<?php
    $command = escapeshellcmd('python3 imu.py');
    $output = shell_exec($command);
    echo $output;
    echo "robot is moving forward";

?>
