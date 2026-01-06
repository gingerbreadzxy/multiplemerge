@echo off
REM Windows - 构建SUMO路网

echo 正在构建SUMO路网...

netconvert --node-files=highway.nod.xml --edge-files=highway.edg.xml --connection-files=highway.con.xml --output-file=highway.net.xml --no-turnarounds true

if %errorlevel% equ 0 (
    echo 路网构建成功！
) else (
    echo 路网构建失败！
)

pause


