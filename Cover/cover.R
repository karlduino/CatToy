mm2in <-
function(mm)
{
  cm <- mm/10
  cm/2.54
}


height <- 2.75
width <-  5.75
pdf("cover.pdf", height=height, width=width)
par(mar=rep(0.1, 4))
plot(0, 0, type="n", xaxt="n", yaxt="n", xlab="", ylab="",
     xlim=c(0, width), ylim=c(0,height), xaxs="i", yaxs="i")

# servos
servo_size <- c("height"=mm2in(23.71), "width"=mm2in(12.11))
y1 <- height/2 - servo_size["height"]/2
y2 <- height/2 + servo_size["height"]/2
x1 <- 0.5
x2 <- x1 + servo_size["width"]


# slots for cables
slot_height <- 0.15
y1n <- y1 - (y1 -slot_height)/2
y2n <- y1n - slot_height

# servos, adjusted
y1p <- y1 + y2n/2
y2p <- y2 + y2n/2
rect(x1, y1p, x2, y2p, lwd=2)
rect(width-x1, y1p, width-x2, y2p, lwd=2)

# slots for cables, adjusted
y1np <- y1n + y2n/2
y2np <- y2n + y2n/2
rect(x1, y1np, x2, y2np, lwd=2)
rect(width-x1, y1np, width-x2, y2np, lwd=2)

# button
m <- width/2
y3 <- (y1 + y1n)/2
points(m, y3, cex=4)
points(m, y3, cex=0.5, pch=16)

# LEDs
x3 <- m - mm2in(8)
x4 <- m + mm2in(8)
y4 <- y3 + (y2 + height)/2 - y2
points(x3, y4, cex=2)
points(x4, y4, cex=2)
points(x3, y4, cex=0.5, pch=16)
points(x4, y4, cex=0.5, pch=16)

# mic
y5 <- height-y3
points(m, y5, cex=4)
points(m, y5, cex=0.5, pch=16)



dev.off()
