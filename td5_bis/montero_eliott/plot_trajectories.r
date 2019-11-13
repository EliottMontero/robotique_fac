#!/usr/bin/env Rscript

# Those libraries should be available at CREMI, otherwise:
# Run R and type:
# install.packages(c("ggplot2","reshape"))
library(ggplot2)
library(reshape)

args = commandArgs(trailingOnly=TRUE)
if (length(args) == 0) {
    stop("Usage: plot_trajectories.r <data_file>")
}

data <- read.csv(args[1])

# Normalizing target_theta in [-pi,pi[
data$target_theta <- (data$target_theta+pi) %% (2*pi) - pi

# Creating data where one line is association of one time and one variable
mdata <- melt(data, id=c("t"))

# Avoiding to have 'factor' columns to allow split
mdata[] <- lapply(mdata, function(x) if (is.factor(x)) as.character(x) else {x})

# Getting separation between type and var
splitted_var <- do.call(rbind, strsplit(mdata$variable, '_'))
mdata$type <- splitted_var[,1]
mdata$var <- splitted_var[,2]

# Drawing graph
g <- ggplot(mdata, aes(x=t,y=value, color=type))
g <- g + geom_line()
g <- g + facet_grid(var~.,scales="free_y")
ggsave("tmp.png")
