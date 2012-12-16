
JAVAFILES := $(wildcard *.java)
CLASSFILES = $(JAVAFILES:%.java=%.class) 


util.jar : $(CLASSFILES)
	jar -cf util.jar $(CLASSFILES)

%.class : %.java
	javac $<
