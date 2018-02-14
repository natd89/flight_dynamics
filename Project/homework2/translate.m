function pnts = translate(pnts,pn,pe,pd)
pnts = pnts + repmat([pn,pe,pd],16,1);
end