import logging
import numpy
import math
import sys

"""
Return true if x is less than or equal to y (with respect to tolerance tol).
"""
def float_le(x, y, tol):
    #TODO: handle degenerate cases (inf e nan)
    return (x == y) or (x-y) <= max(abs(x), abs(y))*tol

"""
lam = lambda
mu = mu
m = number of servers
"""
def p0(lam,mu,c):

    sum = 0.0
    for x in range(0,c):
        f1 = math.pow((lam/mu),x)
        f2 = math.factorial(x)
        val = f1*f2
        sum += val
    f3 = 1.0/math.factorial(c)
    f4 = math.pow((lam/mu),c)
    f5 = (c*mu)/(c*mu-lam)
    sum += f3*f4*f5

    prob0 = 1.0/sum
    return prob0

def pQueue(lam,mu,c):

    f1 = c*math.pow(lam/mu,c)
    f2 = math.factorial(c)
    f3 = c-(lam/mu)
    f4 = p0(lam,mu,c)
    pQ = 1-(f1/(f2*f3))*f4
    #print "PQueue - f1:",f1,"f2:",f2,"f3:",f3,"f4:",f4,"pQ:",pQ
    return pQ


def responseTimeCDF(lam,mu,c,t):

    rho = lam/(mu*c)
    if rho <= 0 or rho >=1 :
        logging.warning( "With c = %d, rho is %f which is unfeasible", c, rho)
        return 0
    retval = 0.0
    f1 = (lam-c*mu+mu*pQueue(lam,mu,c))/(lam-(c-1.0)*mu)
    f2 = 1-math.exp(-mu*t)
    f3 = ((1.0-pQueue(lam,mu,c))*mu)/(lam-(c-1)*mu)
    f4 = 1-math.exp(-(c*mu-lam)*t)
    retval = f1*f2+f3*f4
    #print "responseTimeCDF - f1:",f1,"f2:",f2,"f3:",f3,"f4:",f4,"retval:",retval
    return retval

def responseTimeCDF2(lam,mu,c,t):
    pQ = pQueue(lam,mu,c)
    f1 = pQ*(1-math.exp(-mu*t))
    f2 = ((c*mu-lam)/((c-1)*mu-lam))*(1-math.exp(-mu*t))
    f3 = ((mu)/((c-1)*mu-lam))*(1-math.exp(-(c*mu-lam)*t))
    f4 = (1-pQ)*(f2-f3)
    retval = f1+f4
    return retval


def pi0(lam,mu,m):
    rho = lam/(m*mu)
    part1 = (math.pow(m*rho,m)/math.factorial(m))*(1/(1-rho))
    part2=0.0
    for k in range(0,m):
        part2 +=math.pow(m*rho,k)/math.factorial(k)
    val = 1/(part1+part2)
    return val

def Pm(lam,mu,m):
    rho = lam/(m*mu)
    pai0 = pi0(lam,mu,m)
    pm = (math.pow(m*rho,m)/(math.factorial(m)*(1-rho)))*pai0
    return pm

"""
Compute the average response time for an M/M/m model:
 \frac{C(m, \lambda/\mu)}{m \mu - \lambda} + \frac{1}{\mu}
where:
- C(m, \lambda/\mu) is the Erlang's C formula:
  C(m,\lambda/\mu) = \frac{\left(\frac{(m\rho)^{m}}{m!}\right)\left(\frac{1}{1-\rho}\right)}{\sum_{k=0}^{m-1}\frac{(m\rho)^{k}}{k!} + \left(\frac{(m\rho)^m}{m!}\right)\left(\frac{1}{1-\rho}\right)}
- \rho is the system utilization:
  \rho = \frac{\lambda}{m\mu}
.

Parameters:
- arrival rate \lambda=lam,
- service rate \mu=mu, and
- number of server m=m
.
"""
def avgRespTime_MMC(lam,mu,m):

    rho = lam / (m * mu)
    if m==1:
        val = (1.0/mu)/(1.0-rho)
    else:
        pm = Pm(lam,mu,m)
        avgK = m*rho+(rho/(1-rho))*pm
        val = avgK/lam
    return val

def getMu_MM1(lam,r,perc):

    val = (lam*r-math.log(1-perc))/r
    return val

def respTimeCDF_MM1(lam,mu,r):
    return (1-math.exp(-mu*(1-lam/mu)*r))


def compute(lam, mu, r, tol):

    #error = 0.0000005

    #set initial mu

    # applies the formula with an increasing number m of
    # servers, all having the same service rate mu, until
    # the resoponse time val is <= than the limit r
    m = 0
    #for m in range(1,101):
    while True:
        m += 1

        # Check for stability
        if (lam/(m*mu)) >= 1:
            #print "[percentileTrivedi] model NOT stable"
            continue

        val = avgRespTime_MMC(lam,mu,m)

        # Check for target response time
        #if val <= r:
        #if (r-val)/r <= tol:
        if float_le(val, r, tol):
            logging.debug("lambda: %f, mu: %f, m: %d, RT: %f, limit: %f", lam, mu, m, val, r)
            break
        logging.debug("lambda: %f, mu: %f, m: %d --> RT: %f > limit: %f", lam, mu, m, val, r)

    return m

#if __name__ == "__main__":
def main(lam,mu,r,tol,debug=True):
   # if(len(sys.argv)<4):
   #     print "not enough input parameters"
   #     sys.exit()
    
    #lam = float(sys.argv[1])
    #mu = float(sys.argv[2])
    #r = float(sys.argv[3])

    logging.basicConfig(stream=sys.stderr, format='[%(filename)s:%(funcName)s:%(levelname)s>> %(message)s', level=logging.DEBUG if debug else logging.INFO)

    logging.debug("Input parameters: lambda = %f, mu =%f, r = %f, tol = %f", lam, mu, r, tol)

    #if((int(lam)/mu)>=1):
    if(r < 1/mu):
        logging.warning("model NOT feasible: response time < service time")
        #sys.exit()
        return -1
    
    return compute(lam,mu,r,tol)
