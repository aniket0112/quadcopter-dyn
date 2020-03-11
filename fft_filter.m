function [u1,u2,u3,u4] = fft_filter(tu,fc,t1,t2,t3,t4)
    close all
    Fs = length(tu)/tu(end);
    f = (-length(tu)/2:length(tu)/2-1)*Fs/length(tu);
    y3 = fftshift(fft(t3));
    y2 = fftshift(fft(t2));
    y4 = fftshift(fft(t4));
    y1 = fftshift(fft(t1));
    subplot(2,2,1);
    plot(f,abs(y1));
    title('y1');
    subplot(2,2,2);
    plot(f,abs(y2));
    title('y2');
    subplot(2,2,3);
    plot(f,abs(y3));
    title('y3');
    subplot(2,2,4);
    plot(f,abs(y4));
    title('y4');

    k1 = y1.*(abs(f)<fc)';
    k2 = y2.*(abs(f)<fc)';
    k3 = y3.*(abs(f)<fc)';
    k4 = y4.*(abs(f)<fc)';

    u1 = ifft(ifftshift(k1),'symmetric');
    u2 = ifft(ifftshift(k2),'symmetric');
    u3 = ifft(ifftshift(k3),'symmetric');
    u4 = ifft(ifftshift(k4),'symmetric');

    figure;
    plot(u1,'r');
    hold on
    plot(u2,'g');
    plot(u3,'b');
    plot(u4,'m');

    plot(t1,'r-');
    plot(t2,'g-');
    plot(t3,'b-');
    plot(t4,'m-');
end