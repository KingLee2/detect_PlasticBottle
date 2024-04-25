function T_p_robot = T_P_Robot(Mp,P_robot)
    KK = [667.6397, 0, 322.5541;0, 666.6333, 241.0442;0, 0, 1];
    z = 733.4693;
    Pc = KK^-1*z*Mp;
    Pc = [Pc;0,0,0,1]
    T_p_robot = P_robot*inv(Pc)
end

