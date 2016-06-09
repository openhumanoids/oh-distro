from mrdplot import *

if __name__ == '__main__':

    #m = MRDPLOT()
    #m.fromFile('rs.mrd')
    #m1 = m.retime(0.002);
    #m1.toFile('rs1.mrd')

    m1 = MRDPLOT()
    m2 = MRDPLOT()
    m3 = MRDPLOT()
    m4 = MRDPLOT()

    m1.fromFile('rs.mrd')
    m2.fromFile('cmd.mrd')
    m3.fromFile('qp.mrd')
    m4.fromFile('ft.mrd')

    m5 = mergeMRD(m1, m2, '2.mrd', 0.002)
    m6 = mergeMRD(m5, m3, '3.mrd', 0.002)
    m7 = mergeMRD(m6, m4, '4.mrd', 0.002)

    m7.toFile()

