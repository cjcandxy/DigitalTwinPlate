function ssmodel = dss2ss(dssmodel)

Ad = dssmodel.A;
Bd = dssmodel.B;
Cd = dssmodel.C;
Dd = dssmodel.D;
Ed = dssmodel.E;

Ad = full(Ad);
Bd = full(Bd);
Cd = full(Cd);
Dd = full(Dd);
Ed = full(Ed);

A = Ed\Ad;
B = Ed\Bd;
C = Cd;
D = Dd;

ssmodel = ss(A,B,C,D);

end