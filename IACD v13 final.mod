MODULE TCP_MOVEMENT_CONTROL
    !AccSet
    !RelTool

    !Position
    CONST robtarget HOME_TARGET:=[[596.74,632.46,1457.73],[0.617369,-0.3071,0.679097,0.251739],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]];
    PERS robtarget currentTarget:=[[596.583,632.234,1457.8],[0.617504,-0.307105,0.678984,0.251706],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]];
    !PERS tooldata tool0 := [ TRUE, [[97.4, 0, 223.1], [0.924, 0, 0.383 ,0]], [5, [23, 0, 75], [1, 0, 0, 0], 0, 0, 0]];

    VAR robtarget nextTarget;
    VAR orient orientation;

    VAR pos homeLocation;
    VAR orient homeOrientation;
    VAR robtarget homeTargetEuler;
    VAR confdata homeConfigurationData;
    VAR extjoint homeExternalJoints;
    VAR robtarget homeTarget;

    VAR num coordX;
    VAR num coordY;
    VAR num coordZ;

    VAR num dX;
    VAR num dY;
    VAR num dZ;

    !Joints
    PERS jointtarget currentRobTarget:=[[46.7166,-36.8836,4.621,2.66284,-4.57588,-4.77321],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]];
    CONST jointtarget HOME_JOINT_TARGET:=[[0,0,8,0,20,-3],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]];
    VAR jointtarget jointTargetVar;

    VAR num jointValue1;
    VAR num jointValue2;
    VAR num jointValue3;
    VAR num jointValue4;
    VAR num jointValue5;
    VAR num jointValue6;

    VAR num jointValues{6};

    CONST num JOINT_LIMITS_4400{6,2}:=[[163,-163],[75,-45],[47,-27],[150,-210],[110,-105],[115,-210]];
    CONST num SAFETY_MARGIN:=15;

    !Speed, zone and configuration
    VAR zonedata zone:=z10;
    VAR speeddata speed:=[200,500,5000,1000];

    !Tools
    PERS tooldata gripper;

    !Bounding box
    CONST num BOX_SIZE:=600;

    VAR num maxX;
    VAR num minX;

    VAR num maxY;
    VAR num minY;

    VAR num maxZ;
    VAR num minZ;

    VAR num homeX;
    VAR num homeY;
    VAR num homeZ;

    !Socket messaging
    VAR socketdev temp_socket;
    VAR socketdev client_socket;
    VAR string received_string;
    VAR bool keep_listening:=TRUE;

    CONST byte msgOK{1}:=[1];
    CONST byte msgERR{1}:=[2];
    CONST byte msgDIS{1}:=[3];
    VAR byte msgVAR{1}:=[0];
    VAR num len;
    VAR bool hit_limit:=FALSE;
    VAR num stringToVal{5}:=[0,0,0,0,0];

    !Teach pendant interface
    VAR num answer;

    !Others    
    VAR string testString;
    VAR num counter:=1;

    !arrays of targets
    VAR robtarget targetArray{13}:=[
        [[1216.01,622.69,1125.85],[0.51481,-0.0225538,0.856902,-0.0134199],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],
            [[1712.45,622.67,1125.85],[0.514809,-0.0225518,0.856903,-0.0134251],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],
           [[1216.01,622.69,1125.85],[0.51481,-0.0225538,0.856902,-0.0134199],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],

            [[1319.89,622.69,1609.49],[0.514811,-0.0225527,0.856902,-0.0134173],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],
            [[1606.24,622.67,1609.49],[0.51481,-0.0225521,0.856903,-0.0134222],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],
            [[1319.89,622.69,1609.49],[0.514811,-0.0225527,0.856902,-0.0134173],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],

            [[1229.58,-743.30,1609.49],[0.514811,-0.0225559,0.856902,-0.0134139],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9173,9E+09]],
            [[1392.36,-743.30,1609.49],[0.514811,-0.0225541,0.856902,-0.0134147],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],
            [[1229.58,-743.30,1609.49],[0.514811,-0.0225559,0.856902,-0.0134139],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9173,9E+09]],

            [[1082.49,-743.31,1185.96],[0.514812,-0.0225569,0.856901,-0.0134196],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,-14.9173,9E+09]],
            [[1588.20,-743.27,1185.96],[0.514811,-0.0225601,0.856902,-0.0134085],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],
            [[1082.49,-743.31,1185.96],[0.514812,-0.0225569,0.856901,-0.0134196],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,-14.9173,9E+09]],
            [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]]];

    !Preplanned fixed speed
    PROC Scene1()
        zone:=fine;
        speed:=v2000;
        FOR j FROM 1 TO 3 DO
            FOR i FROM 1 TO Dim(targetArray,1) DO
                MoveJ targetArray{i},speed,zone,tool0;
            ENDFOR
        ENDFOR
    ENDPROC

    !Scaning
    PROC Scene2()
        speed:=v1000;
        !center    
        MoveJ [[596.74,632.46,1457.73],[0.617369,-0.3071,0.679097,0.251739],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;

        Pause;
        zone:=z10;
        speed:=v100;
        FOR i FROM 1 TO 2 DO
            !above    
            MoveJ [[616.76,647.62,1292.36],[0.744635,-0.237227,0.548696,0.296942],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            !center    
            MoveJ [[596.74,632.46,1457.73],[0.617369,-0.3071,0.679097,0.251739],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
            !below   
            MoveJ [[610.00,653.84,1611.79],[0.478272,-0.355261,0.77605,0.206863],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            !center    
            MoveJ [[596.74,632.46,1457.73],[0.617369,-0.3071,0.679097,0.251739],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
            !back
            MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            !center    
            MoveJ [[596.74,632.46,1457.73],[0.617369,-0.3071,0.679097,0.251739],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
            WaitTime(2);
            zone:=z10;
            speed:=v400;
        ENDFOR
    ENDPROC

    PROC Scene3()
    ENDPROC

    !Preplanned variable speed
    PROC Scene4()
        zone:=fine;
        speed:=v2000;
        Pause;
        WHILE TRUE DO
            MoveJ targetArray{counter},speed,zone,tool0;
            !stringToVal{2},500,5000,1000],fine,tool0;

            msgVAR{1}:=4;

            !SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;

            IF counter<Dim(targetArray,1) THEN
                counter:=counter+1;
            ELSE
                counter:=1;
                TPReadFK answer,"Again?","v200","v500","v1000","v2000","no";
                IF answer=1 THEN
                    speed:=v200;
                ELSEIF answer=2 THEN
                    speed:=v500;
                ELSEIF answer=3 THEN
                    speed:=v1000;
                ELSEIF answer=4 THEN
                    speed:=v2000;
                ELSEIF answer=5 THEN
                    GOTO last4;
                ENDIF
            ENDIF
        ENDWHILE

        last4:
        Pause;
    ENDPROC

    !teaching simple figure
    PROC Scene5()
        WHILE TRUE DO
            zone:=fine;
            speed:=v400;
            Square;
            Hint;
            TPReadFK answer,"Did ok?","no","yes","","","";
            IF answer=2 THEN
                zone:=z10;
                speed:=v600;
                SayYes;
                GOTO last5;
            ELSE
                zone:=z10;
                speed:=v600;
                SayNo;
                !contar cuantes veces y aumentar la velocidad?
                WaitTime(1);
            ENDIF
        ENDWHILE

        last5:
        WaitTime(0);
    ENDPROC

    !teach triangle
    PROC Scene6()
        WHILE TRUE DO
            zone:=fine;
            speed:=v500;
            Triangle;
            Hint;
            TPReadFK answer,"Did ok?","no","yes","","","";
            IF answer=2 THEN
                zone:=z10;
                speed:=v600;
                SayYes;
                GOTO last6;
            ELSE
                zone:=z10;
                speed:=v600;
                SayNo;
                !contar cuantes veces y aumentar la velocidad?
                WaitTime(1);
            ENDIF
        ENDWHILE

        last6:
        WaitTime(0);
    ENDPROC

    PROC Scene7()
        zone:=z10;
        speed:=v1000;
        WHILE TRUE DO
            MoveJ [[233.28,589.62,1811.13],[0.680685,-0.335654,0.476373,0.443928],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            MoveJ [[262.47,-576.97,1811.05],[0.674758,0.307181,0.495186,-0.452916],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            MoveJ [[275.43,-605.33,1733.24],[0.654415,0.324286,0.523008,-0.439365],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[259.60,612.35,1733.20],[0.665444,-0.348432,0.507268,0.422444],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            MoveJ [[270.92,639.19,1627.04],[0.636869,-0.371857,0.541084,0.404163],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[415.39,-556.20,1627.07],[0.667587,0.284131,0.591858,-0.351144],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[429.56,-575.09,1417.78],[0.604738,0.315685,0.658244,-0.318356],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            MoveJ [[370.10,615.09,1417.76],[0.601472,-0.370374,0.629114,0.324454],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            MoveJ [[359.68,597.78,1203.78],[0.531467,-0.404672,0.686849,0.286394],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[394.86,-575.11,1203.81],[0.528389,0.358746,0.7119,-0.29207],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            MoveJ [[381.18,-553.35,1102.71],[0.492617,0.371331,0.738566,-0.271959],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            MoveJ [[326.02,587.56,1102.68],[0.490648,-0.43218,0.704706,0.275454],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
            Hint;
            TPReadFK answer,"Did ok?","no","yes","","","";
            IF answer=2 THEN
                zone:=z10;
                speed:=v600;
                SayYes;
                GOTO last7;
            ELSE
                zone:=z10;
                speed:=v600;
                SayNo;
                TPReadFK answer,"Did ok?","no","yes","","","";
                IF answer=2 THEN
                    SayYes;
                    GOTO last7;
                ENDIF
                WaitTime(1);
            ENDIF
        ENDWHILE

        last7:
        WaitTime(1);
    ENDPROC

    PROC Scene8()
        !complicated sequence
        
            zone:=z10;
            speed:=v2000;
            MoveJ [[604.41,635.60,1360.22],[0.581971,-0.319411,0.709702,0.235815],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9174,9E+09]],speed,zone,tool0;
            MoveJ [[565.49,594.47,1865.13],[0.741277,-0.245652,0.54727,0.301097],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[326.29,752.79,1865.12],[0.677219,-0.337537,0.495902,0.426056],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[399.55,922.16,1920.25],[0.677238,-0.33752,0.495917,0.426023],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[421.94,972.46,1249.84],[0.473962,-0.46701,0.684835,0.297091],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[956.50,457.02,1249.86],[0.548006,-0.19755,0.805025,0.112245],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[977.44,467.02,1593.01],[0.677675,-0.171716,0.701321,0.139353],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[900.82,601.67,1593.01],[0.665974,-0.221423,0.687247,0.187462],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[1009.20,393.70,1593.01],[0.682354,-0.145728,0.707176,0.114269],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[838.15,576.77,1194.10],[0.516142,-0.26472,0.800892,0.14864],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[791.23,-639.57,1194.13],[0.50253,0.266999,0.800126,-0.189669],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[818.89,-661.88,1322.75],[0.553157,0.255428,0.765038,-0.208544],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[963.87,-851.88,1332.21],[0.548407,0.272267,0.759212,-0.220719],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[739.90,-654.05,1308.86],[0.548435,0.272223,0.759218,-0.220686],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[995.76,-880.03,1327.30],[0.548406,0.272267,0.759211,-0.220725],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveJ [[947.86,-837.86,1986.09],[0.759622,0.194654,0.540585,-0.304717],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
            MoveAbsJ [[40.5163,7.97711,-15.2967,2.65686,-4.57584,-4.76763],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]]\NoEOffs,speed,zone,tool0;
            MoveJ [[962.43,821.17,1986.04],[0.77331,-0.207521,0.535793,0.268052],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            MoveJ [[1022.55,872.64,1376.29],[0.577331,-0.286871,0.737981,0.199445],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9172,9E+09]],speed,zone,tool0;
            Square;
            Triangle;
            WaitTime(1);
            Hint;
            Pause;
    ENDPROC

    PROC Scene9()
        zone:=z10;
        speed:=v100;
        MoveJ [[388.34,443.85,1286.33],[0.55255,-0.347519,0.715393,0.249262],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        zone:=z50;
        speed:=v400;
        GoCenter;
        Hint;
        Pause;
    ENDPROC

    PROC Scene10()
            zone:=fine;
            speed:=v400;
            MoveJ [[636.47,-281.14,1470.90],[0.687365,0.128261,0.69569,-0.164605],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            MoveJ [[636.47,-580.96,1741.90],[0.687362,0.128258,0.695692,-0.164609],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            MoveJ [[636.47,-850.42,1525.44],[0.687359,0.12826,0.695695,-0.164604],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[636.62,-615.86,1319.86],[0.687358,0.128265,0.695696,-0.164603],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveJ [[636.47,-281.14,1470.90],[0.687365,0.128261,0.69569,-0.164605],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            GoCenter;
            Pause;
            zone:=z10;
        speed:=v100;
        MoveJ [[388.34,443.85,1286.33],[0.55255,-0.347519,0.715393,0.249262],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        GoCenter;
        Pause;
        zone:=z10;
        speed:=v400;
        MoveJ [[636.47,-281.14,1470.90],[0.687365,0.128261,0.69569,-0.164605],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            MoveC [[636.47,-580.96,1741.90],[0.687362,0.128258,0.695692,-0.164609],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],[[636.47,-850.42,1525.44],[0.687359,0.12826,0.695695,-0.164604],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
            MoveC [[636.62,-615.86,1319.86],[0.687358,0.128265,0.695696,-0.164603],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],[[636.47,-281.14,1470.90],[0.687365,0.128261,0.69569,-0.164605],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
            GoCenter;
            Pause;
    ENDPROC

        PROC Scene11()
            zone:=fine;
            speed:=v400;
            MoveJ [[477.18,172.76,1712.71],[0.654398,-0.291404,0.644636,0.267004],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-387.27,1213.81],[0.662896,-0.0397939,0.7473,-0.0229554],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-698.23,1377.44],[0.662893,-0.0397972,0.747303,-0.0229558],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-462.99,1704.97],[0.662891,-0.039796,0.747305,-0.0229547],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
            MoveJ [[574.53,118.29,1204.02],[0.662882,-0.0397908,0.747313,-0.0229557],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
            GoCenter;
            Pause;
            zone:=z10;
        speed:=v100;
        MoveJ [[388.34,443.85,1286.33],[0.55255,-0.347519,0.715393,0.249262],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        GoCenter;
        Pause;
        zone:=z200;
        speed:=v400;
            MoveJ [[477.18,172.76,1712.71],[0.654398,-0.291404,0.644636,0.267004],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-387.27,1213.81],[0.662896,-0.0397939,0.7473,-0.0229554],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-698.23,1377.44],[0.662893,-0.0397972,0.747303,-0.0229558],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]], speed, zone, tool0;
            MoveJ [[519.51,-462.99,1704.97],[0.662891,-0.039796,0.747305,-0.0229547],[-1,-2,1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
            MoveJ [[574.53,118.29,1204.02],[0.662882,-0.0397908,0.747313,-0.0229557],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], speed, zone, tool0;
        GoCenter;
        Pause;
    ENDPROC
    
    PROC Scene12()
        GoCenter;
        zone:=z10;
        speed:=v2000;
        Pause;
        MoveJ [[488.65,434.33,1554.20],[0.893183,-0.14199,0.424363,-0.0444881],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]], speed, zone, tool0;
        WaitTime(1.5);
        MoveJ [[559.90,329.01,1638.02],[0.877556,-0.323153,0.160008,-0.316013],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]], speed, zone, tool0;
        WaitTime(1.5);
        GoCenter;
    ENDPROC
    
    PROC Hint()
        MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[561.69,600.94,1470.18],[0.654394,-0.291397,0.644644,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
        MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[561.69,600.94,1470.18],[0.654394,-0.291397,0.644644,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
        MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
    ENDPROC

    PROC Square()
        MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[477.00,665.06,1470.18],[0.654391,-0.291406,0.644642,0.267005],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[481.61,665.06,1128.73],[0.654392,-0.291407,0.644641,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[481.02,448.09,1128.73],[0.654392,-0.291404,0.644643,0.267003],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
        MoveJ [[481.02,448.10,1389.46],[0.65439,-0.291408,0.644643,0.267003],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
    ENDPROC

    PROC Triangle()
        MoveJ [[477.07,579.39,1439.38],[0.65463,-0.291274,0.64444,0.267052],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[477.07,771.15,1194.02],[0.654629,-0.291275,0.64444,0.267052],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
        MoveJ [[477.07,371.30,1194.02],[0.654628,-0.291277,0.64444,0.267054],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,-14.9169,9E+09]],speed,zone,tool0;
        MoveJ [[477.07,579.39,1439.38],[0.65463,-0.291274,0.64444,0.267052],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
    ENDPROC

    PROC SayYes()
        MoveJ [[552.60,593.13,1527.24],[0.78359,-0.240558,0.48874,0.298751],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[543.09,582.83,1389.56],[0.429331,-0.391966,0.797035,0.163626],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
        MoveJ [[552.60,593.13,1527.24],[0.78359,-0.240558,0.48874,0.298751],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;
        MoveJ [[543.09,582.83,1389.56],[0.429331,-0.391966,0.797035,0.163626],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]],speed,zone,tool0;
        GoCenter;
    ENDPROC

    PROC GoCenter()
        MoveJ [[477.00,505.36,1470.18],[0.654392,-0.291406,0.644642,0.267003],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]],speed,zone,tool0;

        !        !arriba derecha
        !        MoveJ [[546.29,785.98,1912.14],[0.821576,-0.401295,0.401434,0.0531574],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        !        MoveAbsJ [[56.2368,-13.4949,-11.1178,35.6838,-12.1109,-79.0472],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]]\NoEOffs,speed,zone,tool0;
        !        !abajo derecha
        !        MoveJ [[584.96,844.00,1203.36],[0.574931,-0.604556,0.55122,0.0110975],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]],speed,zone,tool0;
        !        MoveAbsJ [[56.2367,-9.15186,29.8353,35.6841,-12.111,-79.0475],[9E+09,9E+09,9E+09,9E+09,-14.9164,9E+09]]\NoEOffs,speed,zone,tool0;
        !        !abajo izq
        !        MoveJ [[738.05,-713.97,1203.39],[0.380591,0.0288785,0.817606,-0.431087],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]],speed,zone,tool0;
        !        MoveAbsJ [[-43.0938,-9.15228,29.8358,35.6851,-12.111,-79.0479],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]]\NoEOffs,speed,zone,tool0;
        !        !arriba izq
        !        MoveJ [[718.01,-695.38,1982.85],[0.585411,0.0474954,0.540704,-0.602228],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9164,9E+09]],speed,zone,tool0;
        !        MoveAbsJ [[-43.0938,-9.1522,-14.5679,35.6855,-12.1111,-79.0487],[9E+09,9E+09,9E+09,9E+09,-14.9164,9E+09]]\NoEOffs,speed,zone,tool0;
        !
        !
        !        MoveJ [[665.94,-505.67,1841.84],[0.600919,0.00999904,0.579184,-0.550765],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        !
        !        MoveJ [[779.25,-588.02,1191.83],[0.405734,-0.0216298,0.818433,-0.4063],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        !
        !        MoveJ [[556.78,801.88,1191.79],[0.574073,-0.605062,0.551561,0.0109774],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        !
        !        MoveJ [[546.28,785.98,1912.14],[0.821576,-0.401295,0.401433,0.0531615],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]],speed,zone,tool0;
    ENDPROC

    PROC SayNo()
        !MoveJ [[467.90,512.74,1459.58],[0.630356,-0.702806,0.216284,-0.248871],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]], v100, zone, tool0;
        MoveJ [[519.53,417.26,1462.41],[0.474575,-0.571905,0.451221,-0.494068],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]],speed,zone,tool0;
        MoveJ [[366.06,543.02,1463.41],[0.687171,-0.724422,-0.0465493,0.0290207],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        MoveJ [[519.53,417.26,1462.41],[0.474575,-0.571905,0.451221,-0.494068],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9165,9E+09]],speed,zone,tool0;
        MoveJ [[366.06,543.02,1463.41],[0.687171,-0.724422,-0.0465493,0.0290207],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9166,9E+09]],speed,zone,tool0;
        MoveJ [[467.90,512.74,1459.58],[0.630356,-0.702806,0.216284,-0.248871],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],speed,zone,tool0;
        GoCenter;
    ENDPROC

    PROC Pause()
        TPReadFK answer,"Go?","yes","","","","";
    ENDPROC

    PROC playSequence()
        !targetArray{1}:=[[-86.62,1131.28,1112.52],[0.000132489,0.861912,-0.507058,0.000106168],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]];
        WHILE TRUE DO

        ENDWHILE
        !        targetArray{1}:=[[-86.62,1131.28,1112.52],[0.000132489,0.861912,-0.507058,0.000106168],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]];
        !        targetArray{2}:=[[-86.62,1131.28,1312.52],[0.000132489,0.861912,-0.507058,0.000106168],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9168,9E+09]];
        !        targetArray{3}:=[[-533.21,1001.46,1312.52],[0.000110604,0.94754,-0.319637,0.000127958],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]];
        !        targetArray{4}:=[[-533.21,1001.46,1112.52],[0.000110604,0.94754,-0.319637,0.000127958],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]];
        !
        !        targetArray{5}:=[[-805.66,798.82,1112.53],[8.91617E-05,0.984701,-0.174254,0.000138242],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]];
        !        targetArray{6}:=[[-805.66,798.82,1312.53],[8.91617E-05,0.984701,-0.174254,0.000138242],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.917,9E+09]];
        !
        !        !        targetArray{6}:=Offs(CRobT(\Tool:=tool0\WObj:=wobj0),0,6*stepSize,stepSize);
        !        !        targetArray{7}:=Offs(CRobT(\Tool:=tool0\WObj:=wobj0),0,7*stepSize,stepSize);
        !        !        targetArray{8}:=Offs(CRobT(\Tool:=tool0\WObj:=wobj0),0,8*stepSize,0);
        !        !        targetArray{9}:=Offs(CRobT(\Tool:=tool0\WObj:=wobj0),0,9*stepSize,0);
        !        !        targetArray{10}:=Offs(CRobT(\Tool:=tool0\WObj:=wobj0),0,10*stepSize,stepSize);
        !
        !        !        FOR index FROM 1 TO Dim(targetArray,1) DO
        !        !            MoveJ targetArray{index},[stringToVal{2},500,5000,1000],fine,tool0;
        !        !        ENDFOR
        !
        MoveJ targetArray{counter},[stringToVal{2},500,5000,1000],fine,tool0;

        msgVAR{1}:=4;

        SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;

        IF counter<Dim(targetArray,1) THEN
            counter:=counter+1;
        ELSE
            counter:=1;
        ENDIF
    ENDPROC

    PROC ChangePosition(bool relative)
        hit_limit:=FALSE;

        IF relative THEN
            ! EXPERIMENTAL
            dX:=stringToVal{2};
            dY:=stringToVal{3};
            dZ:=stringToVal{4};

            coordX:=homeLocation.x+dX;
            coordY:=homeLocation.y+dY;
            coordZ:=homeLocation.z+dZ;
        ELSE
            coordX:=homeLocation.x+stringToVal{2};
            coordY:=homeLocation.y+stringToVal{3};
            coordZ:=homeLocation.z+stringToVal{4};
        ENDIF

        IF coordX>=maxX THEN
            coordX:=maxX;
            hit_limit:=TRUE;
        ENDIF

        IF coordX<=minX THEN
            coordX:=minX;
            hit_limit:=TRUE;
        ENDIF

        IF coordY>=maxY THEN
            coordY:=maxY;
            hit_limit:=TRUE;
        ENDIF

        IF coordY<=minY THEN
            coordY:=minY;
            hit_limit:=TRUE;
        ENDIF

        IF coordZ>=maxZ THEN
            coordZ:=maxZ;
            hit_limit:=TRUE;
        ENDIF

        IF coordZ<=minZ THEN
            coordZ:=minZ;
            hit_limit:=TRUE;
        ENDIF

        !TPWrite "target: x = "+NumToStr(coordX,3)+", y = "+NumToStr(coordY,3)+", z = "+NumToStr(coordZ,3);

        !MoveJ Offs(CRobT(\Tool:=tool0\WObj:=wobj0),stringToVal{2},stringToVal{3},stringToVal{4}),[200,500,5000,1000],zone,tool0\WObj:=wobj0;
        !MoveJ Offs(CRobT(\Tool:=tool0\WObj:=wobj0),homeX,homeY+stringToVal{2},homeZ+stringToVal{3}),[200,500,5000,1000],zone,tool0\WObj:=wobj0;

        IF relative THEN
            MoveJ RelTool(CRobT(\Tool:=tool0\WObj:=wobj0),dX,dY,dZ),vmax,fine,tool0;
        ELSE
            nextTarget:=[[coordX,coordY,coordZ],currentTarget.rot,currentTarget.robconf,currentTarget.extax];
            MoveJ nextTarget,[stringToVal{5},500,5000,1000],zone,tool0\WObj:=wobj0;
        ENDIF

        IF hit_limit THEN
            msgVAR{1}:=5;
        ELSE
            msgVAR{1}:=4;
        ENDIF

        hit_limit:=FALSE;

        !The client needs to receive a byte before sending more data.
        SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;
    ENDPROC

    PROC ChangeJointValues(bool relative)
        hit_limit:=FALSE;

        currentRobTarget:=CalcJointT(CRobT(\Tool:=tool0\WObj:=wobj0),tool0\WObj:=wobj0);

        IF relative THEN
            ! EXPERIMENTAL

            jointValues{1}:=currentRobTarget.robax.rax_1+stringToVal{2};
            jointValues{2}:=currentRobTarget.robax.rax_2+stringToVal{3};
            jointValues{3}:=currentRobTarget.robax.rax_3+stringToVal{4};
            jointValues{4}:=currentRobTarget.robax.rax_4;
            jointValues{5}:=currentRobTarget.robax.rax_5;
            jointValues{6}:=currentRobTarget.robax.rax_6;

            !            jointValue1:=currentRobTarget.robax.rax_1+stringToVal{2};
            !            jointValue2:=currentRobTarget.robax.rax_2+stringToVal{3};
            !            jointValue3:=currentRobTarget.robax.rax_3+stringToVal{4};
            !            jointValue4:=currentRobTarget.robax.rax_4;
            !            jointValue5:=currentRobTarget.robax.rax_5;
            !            jointValue6:=currentRobTarget.robax.rax_6;

        ELSE
            ! experimental
            jointValues{1}:=stringToVal{2};
            jointValues{2}:=stringToVal{3};
            jointValues{3}:=stringToVal{4};
            jointValues{4}:=currentRobTarget.robax.rax_4;
            jointValues{5}:=currentRobTarget.robax.rax_5;
            jointValues{6}:=currentRobTarget.robax.rax_6;
        ENDIF


        !SAFETY_MARGIN
        FOR i FROM 1 TO Dim(jointValues,1) DO
            !            TPWrite "Joint " + NumToStr(i,0) + " limit high = " + NumToStr(JOINT_LIMITS_4400{i,1},0);
            !            TPWrite "Joint " + NumToStr(i,0) + " limit low = " + NumToStr(JOINT_LIMITS_4400{i,2},0);
            IF jointValues{i}>=JOINT_LIMITS_4400{i,1}-SAFETY_MARGIN THEN
                jointValues{i}:=JOINT_LIMITS_4400{i,1}-SAFETY_MARGIN;
                hit_limit:=TRUE;
            ELSEIF jointValues{i}<=JOINT_LIMITS_4400{i,2}+SAFETY_MARGIN THEN
                jointValues{i}:=JOINT_LIMITS_4400{i,2}+SAFETY_MARGIN;
                hit_limit:=TRUE;
            ENDIF
        ENDFOR

        jointTargetVar:=[[jointValues{1},jointValues{2},jointValues{3},jointValues{4},jointValues{5},jointValues{6}],currentRobTarget.extax];
        MoveAbsJ jointTargetVar,[stringToVal{5},500,5000,1000],fine,tool0\WObj:=wobj0;

        IF hit_limit THEN
            msgVAR{1}:=5;
        ELSE
            msgVAR{1}:=4;
        ENDIF

        hit_limit:=FALSE;

        !The client needs to receive a byte before sending more data.
        SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;
    ENDPROC

    PROC ChangeOrientationEuler()
        orientation:=OrientZYX(stringToVal{2},stringToVal{3},stringToVal{4});
        !stringToVal{5};
        nextTarget:=[CPos(\Tool:=tool0\WObj:=wobj0),orientation,homeConfigurationData,homeExternalJoints];
        MoveJ nextTarget,vmax,zone,tool0\WObj:=wobj0;

        msgVAR{1}:=4;

        !The client needs to receive a byte before sending more data.
        SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;
    ENDPROC

    PROC ChangeOrientationQuaternion()
        ! unfinished
        orientation:=[stringToVal{2},stringToVal{3},stringToVal{4},stringToVal{5}];
        nextTarget:=[CPos(\Tool:=tool0\WObj:=wobj0),orientation,homeConfigurationData,homeExternalJoints];
        MoveJ nextTarget,vmax,zone,tool0\WObj:=wobj0;

        msgVAR{1}:=4;

        !The client needs to receive a byte before sending more data.
        SocketSend client_socket\Data:=msgVAR\NoOfBytes:=1;
    ENDPROC

    PROC main()
        GoHome;
        currentTarget:=CRobT(\Tool:=tool0\WObj:=wobj0);

        Scene1;
        Scene2;
        Scene3;
        Scene4;
        Scene5;
        Scene6;
        Scene7;
        Scene8;
        Scene9;
        Scene10;
Scene11;
Scene12;
        
        Break;
        !        gripper:=[TRUE,[[0,0,0],[0,0,0.382637610167511, 0.9238506372785592]],load0];
        homeTarget:=currentTarget;

        homeLocation:=homeTarget.trans;
        homeOrientation:=homeTarget.rot;
        !VAR robtarget homeTargetEuler;
        homeConfigurationData:=homeTarget.robconf;
        homeExternalJoints:=homeTarget.extax;

        !VAR robtarget homeTarget:=[homeLocation,homeOrientation,home_configurationData,home_externalJoints];

        nextTarget:=homeTarget;
        orientation:=homeOrientation;

        !move home considering configuration data, then turn off the checking
        homeTargetEuler:=[homeLocation,OrientZYX(58.7,29.9,90.6),homeConfigurationData,homeExternalJoints];
        !MoveJ homeTargetEuler,vmax,fine,tool0\WObj:=wobj0;
        !MoveJ homeTarget,vmax,fine,tool0\WObj:=wobj0;

        maxX:=homeLocation.x+BOX_SIZE/2;
        minX:=homeLocation.x-BOX_SIZE/2;

        maxY:=homeLocation.y+BOX_SIZE/2;
        minY:=homeLocation.y-BOX_SIZE/2;

        maxZ:=homeLocation.z+BOX_SIZE/2;
        minZ:=homeLocation.z-BOX_SIZE/2;

        !MoveL Offs(CRobT(\Tool:=tool0\WObj:=wobj0),homeX,homeY+stringToVal{2},homeZ+stringToVal{3}),[200,500,5000,1000],zone,tool0\WObj:=wobj0;

        !TPWrite "wobj0 x="+NumToStr(homeX,2)+", y="+NumToStr(homeY,2)+", z = "+NumToStr(homeZ,2);
        !TPWrite "wobjIACD x="+NumToStr(currentTarget.trans.x,2)+", y="+NumToStr(currentTarget.trans.y,2)+", z = "+NumToStr(currentTarget.trans.z,2);

        !            VAR pos homeLocation;
        !    VAR orient homeOrientation;
        !    VAR robtarget homeTargetEuler;
        !    VAR robtarget homeTarget;

        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),100,0,0),vmax,fine,tool0;
        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),-100,0,0),vmax,fine,tool0;
        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),0,100,0),vmax,fine,tool0;
        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),0,-100,0),vmax,fine,tool0;
        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),0,0,100),vmax,fine,tool0;
        !        MoveL RelTool(CRobT(\Tool:=gripper\WObj:=wobj0),0,0,-100),vmax,fine,tool0;

        !        ConfL\Off;
        !        ConfJ\Off;

        !        TPReadFK answer,"Select Zone","fine","z0","z1","z5","z10";
        !        IF answer=1 THEN
        !            zone:=fine;
        !        ELSEIF answer=2 THEN
        !            zone:=zone;
        !        ELSEIF answer=3 THEN
        !            zone:=z100;
        !        ELSEIF answer=4 THEN
        !            zone:=z150;
        !        ELSEIF answer=5 THEN
        !            zone:=z200;
        !        ENDIF

        zone:=zone;

        !        testString:="hello how are you?";
        !        len:=StrLen(testString);
        !        testByte := StrToByte(testString);
        !        
        !        FOR i FROM 1 TO len DO
        !                TPWrite "Character at: " + NumToStr(i,0) + " is " +  StrPart(testString,i,1);
        !        ENDFOR

        SocketCreate temp_socket;
        SocketBind temp_socket,"128.2.109.111",6000;
        !111",6000;
        SocketListen temp_socket;
        ! Waiting for a connection request
        SocketAccept temp_socket,client_socket;

        !Say OK to let the client know I'm alive
        SocketSend client_socket\Data:=msgOK\NoOfBytes:=1;

        WHILE keep_listening DO
            ! Communication
            SocketReceive client_socket\Str:=received_string;
            len:=StrLen(received_string);
            !TPWrite "Client wrote: "+NumToStr(len,0)+" characters";
            !TPWrite "Client wrote this: "+received_string;

            !Understand message
            IF received_string="Disconnect" THEN
                ! Shutdown the connection
                TPWrite "Shutdown acknowledged";
                keep_listening:=FALSE;
                !SocketSend client_socket\Data:=msgDIS\NoOfBytes:=1;
                !SocketSend client_socket\Str:="Shutdown acknowledged";
                SocketClose client_socket;
                !NEED TO BREAK OUT OF THE WHILE HERE
                !SocketSend socket1\Data:=msg_dc\NoOfBytes:=1;
            ELSE
                IF StrToVal(received_string,stringToVal) THEN
                    !look at parsing
                    TPWrite received_string+" Step parsed into "+NumToStr(stringToVal{1},0)+","+NumToStr(stringToVal{2},3)+","+NumToStr(stringToVal{3},3)+","+NumToStr(stringToVal{4},3)+","+NumToStr(stringToVal{5},3);
                    !+","+NumToStr(stringToVal{5},2);
                    !SocketSend client_socket\Data:=msgOK\NoOfBytes:=1;
                ELSE
                    TPWrite "StrToVal("+received_string+") returned false";
                    !SocketSend client_socket\Data:=msgERR\NoOfBytes:=1;
                ENDIF

                currentTarget:=CRobT(\Tool:=tool0\WObj:=wobj0);

                IF stringToVal{1}=1 THEN
                    ChangePosition(TRUE);
                ELSEIF stringToVal{1}=2 THEN
                    !ChangeOrientationQuaternion;
                ELSEIF stringToVal{1}=3 THEN
                    !ChangeOrientationEuler;
                ELSEIF stringToVal{1}=4 THEN
                    ChangeJointValues(FALSE);
                ELSEIF stringToVal{1}=5 THEN
                    playSequence;
                ELSEIF stringToVal{1}=6 THEN
                    ChangePosition(FALSE);
                ENDIF

                stringToVal:=[0,0,0,0,0];
            ENDIF

            received_string:="";
        ENDWHILE

        SocketClose temp_socket;
        TPWrite "Have a good one!";
    ENDPROC

    PROC GoHome()
        !GO_HOME_4400;
        MoveJ HOME_TARGET,[200,500,5000,1000],fine,tool0;
        !MoveAbsJ HOME_JOINT_TARGET,[200,500,5000,1000],fine,tool0\WObj:=wobj0;
        !MoveJ [[467.90,512.74,1459.58],[0.630356,-0.702806,0.216284,-0.248871],[0,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,-14.9167,9E+09]],v100,zone,tool0;
    ENDPROC

    PROC UpdateCurrentTarget()
        currentTarget:=CRobT(\Tool:=tool0\WObj:=wobj0);
        currentRobTarget:=CalcJointT(CRobT(\Tool:=tool0\WObj:=wobj0),tool0\WObj:=wobj0);
    ENDPROC
ENDMODULE