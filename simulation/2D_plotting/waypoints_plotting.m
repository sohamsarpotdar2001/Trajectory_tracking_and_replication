cam = webcam();

img = snapshot(cam);
frameSize = size(img);
frameCount = 0;
x = zeros(0,2);
count1 =1;
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

while frameCount < 4000
    img = snapshot(cam);
    diff_img = imsubtract(img(:,:,3),rgb2gray(img));
    diff_img = medfilt2(diff_img,[3,3]);
    diff_img = imbinarize(diff_img,0.18);
    diff_img = bwareaopen(diff_img,1200);
    bw=bwlabel(diff_img,8);
    stats=regionprops(logical(bw),'BoundingBox','Centroid');
    for object=1:length(stats)
        bbox = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        bboxPoints = bbox2points(bbox(1, :));
        bboxPolygon = reshape(bboxPoints', 1, []);
%         plot(bc(1),bc(2),'-m+');
        x(count1,:) = [round(bc(1)),round(bc(2))];
        
        plot(x(:,1),x(:,2),'-m+');
%         a=text(bc(1)+15,bc(2),strcat('X: ',num2str(round(bc(1))),'  Y :',num2str(round(bc(2)))));
        count1=count1+1;
        img = insertShape(img, 'polygon', bboxPolygon, 'LineWidth', 3);
        img = insertMarker(img, bc, "plus", "Color", "red");
    end
    
    frameCount = frameCount + 1;
    step(videoPlayer, img);
end

clear cam;
release(videoPlayer);