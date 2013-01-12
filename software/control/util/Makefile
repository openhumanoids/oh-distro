
JAVAFILES := $(wildcard *.java)
CLASSFILES = $(JAVAFILES:%.java=%.class) 


drc_control_util.jar : $(CLASSFILES)
	jar -cf drc_control_util.jar $(CLASSFILES)

%.class : %.java
	javac $<

clean:
	-rm *.class
	-rm drc_control_util.jar
