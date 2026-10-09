FNR==NR {
  if ($1=="translation:") b="t"; else if ($1=="rotation:") b="r";
  if (b=="t" && $1=="x:") tx=$2; if (b=="t" && $1=="y:") ty=$2; if (b=="t" && $1=="z:") tz=$2;
  if (b=="r" && $1=="x:") qx=$2; if (b=="r" && $1=="y:") qy=$2; if (b=="r" && $1=="z:") qz=$2; if (b=="r" && $1=="w:") qw=$2;
  next
}
FNR==1 {
  a = th * 3.14159265358979 / 180; c = cos(a); s = sin(a); ch = cos(a/2); sh = sin(a/2);
  ux = tx - px; uz = tz - pz;
  nx = c*ux + s*uz + px + dx;  ny = ty + dy;  nz = -s*ux + c*uz + pz + dz;
  nqx = ch*qx + sh*qz;  nqy = ch*qy + sh*qw;  nqz = ch*qz - sh*qx;  nqw = ch*qw - sh*qy;
  n = sqrt(nqx*nqx + nqy*nqy + nqz*nqz + nqw*nqw);  nqx /= n; nqy /= n; nqz /= n; nqw /= n;
}
{
  if ($1=="translation:") b2="t"; else if ($1=="rotation:") b2="r";
  if (b2=="t" && $1=="x:") { printf "    x: %.12f\n", nx; next }
  if (b2=="t" && $1=="y:") { printf "    y: %.12f\n", ny; next }
  if (b2=="t" && $1=="z:") { printf "    z: %.12f\n", nz; next }
  if (b2=="r" && $1=="x:") { printf "    x: %.12f\n", nqx; next }
  if (b2=="r" && $1=="y:") { printf "    y: %.12f\n", nqy; next }
  if (b2=="r" && $1=="z:") { printf "    z: %.12f\n", nqz; next }
  if (b2=="r" && $1=="w:") { printf "    w: %.12f\n", nqw; next }
  print
}
