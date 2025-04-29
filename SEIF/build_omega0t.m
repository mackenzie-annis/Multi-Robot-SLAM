function omega_0t = build_omega0t(omega, F_xmo)
    omega_0t = F_xmo * (F_xmo'* omega * F_xmo) * F_xmo';
    %omega_0t = F_xmo * omega * F_xmo';
end