clc;clear;
   % Начальные данные
    while 1
        % Параметры симулинка
        LaunchTime = 0;
        StopTime = 0.1;
        ChargeTime = 0.002;

        % резистивные составляющие
        Resistance = 22;

        % Параметры источника питания
        Voltage = 50;

        % Параметры конденсатора
        Capasity = 688e-6;
        CapasitorVoltage = 1;

        % Массивы параметров катушек
        InductorGap = 58/1000;
        CounterCoil = 10;
        InductorLength = zeros(CounterCoil+1,1);
        InductorRadius = zeros(CounterCoil+1,1);
        QuantityCoil = zeros(CounterCoil+1,1);
        InductorStartPosition = zeros(CounterCoil+1,1);
        Inductance = zeros(CounterCoil+1,1);
        % Параметры катушек
        for a = 1:CounterCoil+1
            InductorLength(a) = 58/1000;
            InductorRadius(a) = 8.5/1000;
            QuantityCoil(a) = 2400;
            InductorStartPosition(a) = ((a-1)*(InductorLength(a)+InductorGap));
            Inductance(a) = 5420e-6;
        end
        % Мнимая послепоследняя катушка
        InductorStartPosition(CounterCoil+1) = 1000;

        % парметры снаряда
        BulletRadius = 5/1000;
        mu = 2500;
        BulletLength = InductorLength(1);
        StartCoordinates = InductorStartPosition(1)-InductorLength(1)*0.4;
        Plotnost = 7800;
        Weight = pi*BulletLength*BulletRadius^2*Plotnost;
        MaxB = 2.12;
        BulletSquareCut = pi*BulletRadius^2;

        % Остальные
        X = InductorLength(1)*1000/2;
        k = 0;
        mu0 = 4*pi*10^-7;
        CoilAccuracy = fix(QuantityCoil(a));
        BulletAccuracy = 50;
        StartSpeed=0;
        StartTime = 1;
        break
    end
    
    % Начальные параметры ключей для успешного запуска кода
    while 1
            SupportTimeArray = linspace(LaunchTime, StopTime, 578)';
            SwitchTime = zeros(length(SupportTimeArray),1+1+CounterCoil);
            SwitchTime(:,1) = SupportTimeArray;
            SwitchTrigger = zeros(1,2+CounterCoil*2);
            SwitchTrigger(1) = LaunchTime;
            SwitchTrigger(2) = ChargeTime;
            SwitchTrigger(3) = ChargeTime;

            for a = 4:2+CounterCoil*2
                SwitchTrigger(a) = StopTime;
            end

    
            % Формирование сигнала для зарядки
            for a = 1:1+CounterCoil
                for b = 1:length(SupportTimeArray)
                    if ((SupportTimeArray(b)>SwitchTrigger(a*2-1))&&(SupportTimeArray(b)<SwitchTrigger(a*2)))
                    SwitchTime(b,a+1) = 1;
                    else 
                        SwitchTime(b,a+1) = 0;
                    end
                    
                end
            end
            break
    end
    
    % Запуск Simulink модели и запись токов
    while 1

        sim('DiplomSimAnalog2Reserve.slx');
        ToqueDischargeTimeArray = ans.tout;
        ToqueDischargeArray = zeros(length(ToqueDischargeTimeArray),CounterCoil);
        for a=1:CounterCoil
            for b=1:length(ToqueDischargeTimeArray)
                ToqueDischargeArray(b,a) = ans.I.signals.values(b,a);
            end
        end
        break
    end
    
    % Параметры ключей
    while 1
        
            SwitchTime = zeros(length(ToqueDischargeTimeArray),1+CounterCoil+1);
            SwitchTime(:,1) = ToqueDischargeTimeArray;
            SwitchTrigger = zeros(1,2+CounterCoil*2);
            SwitchTrigger(1) = LaunchTime;
            SwitchTrigger(2) = ChargeTime;
            SwitchTrigger(3) = ChargeTime;

            for a = 4:2+CounterCoil*2
                SwitchTrigger(a) = StopTime;
            end

    
            % Формирование сигналов
            for a = 1:1+CounterCoil
                for b = 1:length(ToqueDischargeTimeArray)
                    if ((ToqueDischargeTimeArray(b)>SwitchTrigger(a*2-1))&(ToqueDischargeTimeArray(b)<SwitchTrigger(a*2)))
                    SwitchTime(b,a+1) = 1;
                    else 
                        SwitchTime(b,a+1) = 0;
                    end
                    
                end
            end
            break
    end
    % Установка расположения катушек и снаряда
    while 1

        for a = 1:CounterCoil
            for b = 1:CoilAccuracy
                InductorLocation(b,a) = InductorStartPosition(a) + (b-1) * (InductorLength(a)/CoilAccuracy) + (InductorLength(a)/CoilAccuracy) / 2;
            end
        end

        for b = 1:BulletAccuracy
            BulletLocation(b) = (b-1) * (BulletLength/BulletAccuracy) + (BulletLength/BulletAccuracy) / 2;
        end

        break
    end
     
    % Создание массивов
    while 1
        
        X_array = zeros(4*X+2*k*X,1);
        B0B_array = zeros(4*X+2*k*X,1);
        B0B_Full_array = zeros(fix(((InductorLength(1)*(CounterCoil+1)+InductorGap*(CounterCoil-1)))*1000)+1,1);
        H_Real_array = zeros(length(B0B_array),1);
        dH_array = zeros(length(B0B_array),1);
        break
    end
    % Начало моделирования
    while 1

        for b = 1:2*X + k*X
            distance = (b-X)/1000 - k*X/1000;
            B0B = 1-((1-(distance/(InductorRadius(1)^2+distance^2)^0.5))/2);
            B0B_array(b) = B0B;
            X_array(b) = b/1000;
            Continue = b;
        end
        for b = 1:2*X + k*X
            distance = (b-X)/1000;
            B0B = (1-(distance/(InductorRadius(1)^2+distance^2)^0.5))/2;
            B0B_array(b+Continue) = B0B;
            X_array(b+Continue) = (b+2*X)/1000;
        end
        for a = 1:CounterCoil
            for b = 1:length(B0B_array)
                B0B_Full_array(fix((InductorLength(1)+InductorGap)*1000)*(a-1)+b) = B0B_array(b);
            end
        end
        plot(B0B_Full_array);

       for a = 1:length(B0B_Full_array)
            if a == 1
                dH = 0;
            else
                dH = (B0B_Full_array(a)-B0B_Full_array(a-1))/length(B0B_Full_array);
            end
            dH_array(a) = dH;
       end
        plot(dH_array);
       break
    end
