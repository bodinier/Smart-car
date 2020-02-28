function rgyb = LAB2GBYBImage(lab)
rgyb = lab(:,:,1)*(lab(:,:,2)+lab(:,:,3));
end
