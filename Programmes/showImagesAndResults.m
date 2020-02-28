function y = showImagesAndResults(start,stop, directory)
h = figure();
subplot(1,1,1);
i=start;
   while i<stop
    nom=sprintf('%s/frame_%06d.jpg',directory,i);%On d?fini le nom de l'image que l'on va afficher gr?ce ? un sprintf
    rgb=imread(nom);
    
    rgb = deformatImages(rgb); %Cette fonction va convertir l'image en double entre 0 et 1 en transformant d'abord les uint8 en double puis elle mets le minimum de la photo ? 0 et et le maximum ? 1
    %lab = RGB2LABImage(rgb);
    imagesc(rgb);%On affiche cette image 
    detectMaxima2(rgb);
    i=i+1;%On incr?ment de 1 la valeur i utilis?e dans le printf, c'est ? dire qu'on passe ? l'image suivante
    pause(0.05);%On fait une pause avant d'afficher l'image suivante
   end
    close(h);
end

function [x,y] = detectMaxima2(RGB)
thresholdR = 0.9; %seuil pour le feu rouge
thresholdG =0.995; %seuil pour le feu vert
thresholdB = 0.10; % seuil pour le orange

res = restriction(RGB); %On selectionne seulement une portion de l'image a traiter
res = deformatImages(res);

R = res(:, :, 1);
G = res(:, :, 2);
%On cree la matrice des maxima par colonne 
[maxValsR, yR] = max(R);%maxVals vecteur ligne de taille 1x427 et yR vecteur ligne de taille
[maxValsG, yG] = max(G);
x = 1;
while x<427
    if maxValsR(1, x) > thresholdR && res(yR(x), x, 3)<0.25 && res(yR(x), x, 1) > thresholdR
        rectangle('Position', [214+x-5, yR(1,x)-5, 15, 15], 'EdgeColor', 'r', 'LineWidth', 3);
        x=x+2;
    end
    
    if maxValsG(1, x) > 0.9 && res(yG(x), x, 3)>0.6 && res(yR(x), x, 1) < 0.6
        rectangle('Position', [214+x-5, yR(1,x)-5, 15, 15], 'EdgeColor', 'g', 'LineWidth', 3);
        x=x+2;
    end
    x=x+1;
end
end

function mat = restriction(img)
    %Cette fonction renvoie une restriction d une image  : on selectionne
    %seulement une partie de l'image (en haut a droite dans notre cas).
    mat = zeros(160, 427, 3);
    mat(:, :, 1) = img(1:160, 214:640, 1);
    mat(:, :, 2) = img(1:160, 214:640, 2);
    mat(:, :, 3) = img(1:160, 214:640, 3);
end



function newImg = deformatImages(img)
    %converts images into doubles between 0 and 1
    %img : array (it can be uint8, float, etc, a vector, a multimensional matrix)
    %newImg : array of the same size but with double values between 0 and 1
    newImg = double(img); %convert from uint8 (or any other format) to double;
    newImg = newImg - min(newImg(:)); % make the minimum value of the image zero
    newImg = newImg/max(newImg(:)); %make the maximum value of the image 1;

end

function lab = RGB2LABImage(rgb)   
    rgbl = RGBs2RGBLinearImage(rgb);
    xyz = RGBLinear2XYZImage(rgbl);
    lab = XYZ2LABImage(xyz);
end
 function rgbl=RGBs2RGBLinearImage(rgb)

 rgl=zeros(480,640,3);
 for i=1:480
     for j=1:640
         rgb_ij=[ 0 0 0];%On cree un vecteur qui va comporter toutes les intensites de rouge vert bleu pour le pixel i,j de la matrice 1
         rgb_ij(1)=rgb(i,j,1);%Matrice rouge
         rgb_ij(2)=rgb(i,j,2);%Matrice vert
         rgb_ij(3)=rgb(i,j,3);%Matrice bleu 
         rgb_ij=RGBs2RGBLinearPixel(rgb_ij);%On lin?arise 
         rgbl(i,j,1)=rgb_ij(1);%on mets les nouvelles valeurs dans le vecteur rgbl contenant les intensit?s lin?aris?es
         rgbl(i,j,2)=rgb_ij(2);
         rgbl(i,j,3)=rgb_ij(3);
     end
 end
 end
 
 


function xyz = RGBLinear2XYZImage(rgbl)%Le principe est le m?me que pour la fonction pr?c?dente 

 xyz=zeros(480,640,3);
 for i=1:480
     for j=1:640
         rgbl_ij=[ 0 0 0];
         rgbl_ij(1)=rgbl(i,j,1);
         rgbl_ij(2)=rgbl(i,j,2);
         rgbl_ij(3)=rgbl(i,j,3);
         rgbl_ij=RGBLinear2XYZPixel(rgbl_ij);
         xyz(i,j,1)=rgbl_ij(1);
         xyz(i,j,2)=rgbl_ij(2);
         xyz(i,j,3)=rgbl_ij(3);
     end
 end
end

function lab = XYZ2LABImage(xyz) 
 lab=zeros(480,640,3);
 for i=1:480
     for j=1:640
         lab_ij=[ 0 0 0];
         lab_ij(1)=xyz(i,j,1);
         lab_ij(2)=xyz(i,j,2);
         lab_ij(3)=xyz(i,j,3);
         lab_ij=RGBLinear2XYZPixel(lab_ij);
         lab(i,j,1)=lab_ij(1);
         lab(i,j,2)=lab_ij(2);
         lab(i,j,3)=lab_ij(3);
     end
 end
end

function nonlinearity = f(t)
if t>((6/29)^3)
        nonlinearity = t^(1/3);
else
        nonlinearity  = 7.787*t+16/116;
end
end

function lab = RGB2LABPixel(rgb)   
    rgbl = RGBs2RGBLinearPixel(rgb);
    xyz = RGBLinear2XYZPixel(rgbl);
    lab = XYZ2LABPixel(xyz);
end


function rgbl = RGBs2RGBLinearPixel(rgb)
    T = 0.04045;
    rgbl = zeros(1,3);
    for i = 1:3
        if rgb(i) <T
            rgbl(i) = rgb(i)/12.92; 
        else
            rgbl(i) = ((rgb(i)+0.055)/1.055)^2.4;
        end
    end
end



function xyz = RGBLinear2XYZPixel(rgbl)
    M = [0.4124     0.3576      0.1805;
        0.2126      0.7152      0.0722;
        0.0193     0.1192      0.9505];
    xyz = M*rgbl';
    xyz = xyz';
end


function lab = XYZ2LABPixel(xyz) 
    D65white = [0.9505     1       1.0890];
    xyzn = xyz./D65white;
    lab = [ 116*f(xyzn(2))-16;
             500*(f(xyzn(1))-f(xyzn(2)));
             200*(f(xyzn(2))-f(xyzn(3)))];
end

function rgyb = LAB2GBYBImage(lab)
rgyb = lab(:,:,1)*(lab(:,:,2)+lab(:,:,3));
end


