from mrdplot import *

if __name__ == '__main__':

    #m = MRDPLOT()
    #m.fromFile('rs.mrd')
    #m1 = m.retime(0.002);
    #m1.toFile('rs1.mrd')

    m1 = MRDPLOT()
    m2 = MRDPLOT()

    m2.fromFile('cmd.mrd')
    m1.fromFile('rs.mrd')

    m3 = mergeMRD(m1, m2, 'both.mrd', 0.002)
    m3.toFile()

