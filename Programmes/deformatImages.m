function newImg = deformatImages(img)
    %converts images into doubles between 0 and 1
    %img : array (it can be uint8, float, etc, a vector, a multimensional matrix)
    %newImg : array of the same size but with double values between 0 and 1
    newImg = double(img); %convert from uint8 (or any other format) to double;
    newImg = newImg - min(newImg(:)); % make the minimum value of the image zero
    newImg = newImg/max(newImg(:)); %make the maximum value of the image 1;

end