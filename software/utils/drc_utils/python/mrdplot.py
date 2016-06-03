from array import array

class MRDPLOT:

    def __init__(self):
        self.name = ''
        self.channel_names = []
        self.channel_units = []
        self.n_points = 0
        self.n_channels = 0
        self.freq = 0
        self.data = array('f')  

    
    def checkSize(self):
        ret = len(self.channel_names) == len(self.channel_units)
        ret = ret and len(self.channel_names) == self.n_channels
        ret = ret and len(self.data) == self.n_points * self.n_channels
        return ret


    def fromFile(self, n):
        self.name = n
        fin = open(self.name, 'r')

        # read header
        ln = fin.readline()
        ln = ln.rstrip()
        fields = ln.split(' ')
        
        self.n_channels = int(fields[1])
        self.n_points = int(fields[2])
        self.freq = int(fields[3])

        assert(self.n_points * self.n_channels == int(fields[0]))

        # read channels
        for i in xrange(self.n_channels):
            ln = fin.readline()
            ln = ln.rstrip()
            fields = ln.split(' ')
            self.channel_names.append(fields[0])
            self.channel_units.append(fields[1])

        ln = fin.readline()
        ln = fin.readline()

        # read data
        self.data.fromfile(fin, self.n_points * self.n_channels)
        self.data.byteswap()
        assert(len(self.data) == self.n_points * self.n_channels)

        fin.close()
        
        assert(self.checkSize())

    def toFile(self, n=None):
        if n == None:
            n = self.name
        out = open(n, 'w')
        out.write(str(self.n_channels*self.n_points) + ' ')
        out.write(str(self.n_channels) + ' ')
        out.write(str(self.n_points) + ' ')
        out.write(str(self.freq) + '\n')

        for i in xrange(0, self.n_channels):
            out.write(self.channel_names[i] + ' ' + self.channel_units[i] + '\n')

        out.write('\n\n')
        float_data = self.data
        float_data.byteswap()
        float_data.tofile(out)

        out.close()

    
    def interpAtTime(self, t):
        if (self.n_points < 2):
            return list(self.data)
        
        if (t <= self.data[0]):
            idx0 = 0
            idx1 = 1
        elif (t >= self.data[(self.n_points-1)*self.n_channels]):
            idx0 = self.n_points-2
            idx1 = self.n_points-1
        else:
            idx0, idx1 = self._getIdx(t, 0, self.n_points-1)
        
        assert(idx1 < self.n_points and idx0 >= 0)

        t0 = self.data[idx0*self.n_channels]
        t1 = self.data[idx1*self.n_channels]

        assert(t >= t0 and t <= t1 and not t1 == t0)

        # linear interp all the channels at t
        alpha = (t - t0) / (t1 - t0)
        data = [t]
        for i in xrange(1, self.n_channels):
            n0 = self.data[idx0*self.n_channels + i]
            n1 = self.data[idx1*self.n_channels + i]
            data.append(alpha*n1 + (1-alpha)*n0)
        
        return data


    def retime(self, dt):
        newmrd = self;
        newmrd.freq = 1. / dt

        t = self.data[0]
        data = []
        ctr = 0

        while True:
            d = self.interpAtTime(t)
            data.extend(d)
            ctr = ctr + 1

            t = t + dt

            if t >= self.data[self.n_channels*(self.n_points-1)]:
                break

        newmrd.n_points = ctr
        newmrd.data = array('f', data)

        assert(newmrd.checkSize())

        return newmrd
    
    
    def _getIdx(self, t, start, end):
        mid = (start+end)/2;
        
        assert(end > start)
        if (end - start == 1):
            return start, end

        if (t < self.data[mid*self.n_channels]):
            return self._getIdx(t, start, mid)
        elif (t > self.data[mid*self.n_channels]):
            return self._getIdx(t, mid, end)
        else:
            return mid, mid+1
        

def mergeMRD(m1, m2, newname, dt):
    # assume time is the first idx
    min_t = max(m1.data[0], m2.data[0])
    max_t = min(m1.data[m1.n_channels*(m1.n_points-1)], m2.data[m2.n_channels*(m2.n_points-1)])
    
    if (max_t <= min_t):
        return None

    t = min_t
    new_data = []
    ctr = 0

    while True:
        data1 = m1.interpAtTime(t);
        data2 = m2.interpAtTime(t);

        # first element is time
        data2.pop(0)
        
        new_data.extend(data1)
        new_data.extend(data2)
        ctr = ctr + 1

        t = t + dt
        if t >= max_t:
            break

    # rewrite 
    ret = MRDPLOT()
    ret.name = newname
    
    ret.channel_names = m1.channel_names
    tmp = m2.channel_names
    tmp.pop(0)
    ret.channel_names.extend(tmp)

    ret.channel_units = m1.channel_units
    tmp = m2.channel_units
    tmp.pop(0)
    ret.channel_units.extend(tmp)

    ret.n_points = ctr
    ret.n_channels = m1.n_channels + m2.n_channels - 1
    ret.freq = 1. / dt
    ret.data = array('f', new_data)

    assert(ret.checkSize())

    return ret
 
