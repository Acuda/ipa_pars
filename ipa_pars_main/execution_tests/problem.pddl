(define (problem cob-test-problem-01)
	(:domain cob-test-domain-01)
	(:objects 
 		 ;;; available objects 
 
 		 ;;; fixed locations 
		 room-7011 room-5003 room-7009 room-5002 
		 room-1003 room-1002 room-1001 room-7012 
		 room-7017 room-1004 room-1005 room-1006 
		 room-5004 room-7013 room-7014 room-7015 
		 room-7016 room-6001 room-7018 room-2002 
		 room-2001 room-2005 room-2004 room-2003 
		 room-6004 room-2006 room-6002 room-6003 
		 room-7019 room-7020 room-7021 room-7022 
		 room-7023 room-4001 room-4002 room-4003 
		 room-4004 room-2007 room-6005 room-7024 
		 room-7025 room-7026 room-3001 room-3002 
		 room-4005 room-4006 room-5001 room-7001 
		 room-8001 room-7004 room-7003 room-7005 
		 room-7002 room-7006 room-7007 room-7008 
		 room-7010 room-10001 room-10002 room-9001 
		 room-9002 room-9003 room-9004 room-9005 
		 room-9006 room-10003 room-10004 room-9007 
		 room-9008 room-9009 room-10005 room-10006 
		 room-9010 room-9011 room-9012 room-10007 
		 room-10008 
 
 		 ;;; movable things 
 
		 the-cake 
		 cob4-1)


	(:init  
 		 ;;; transitions for room-7011
		(trans room-7011 room-7009)
		(trans room-7011 room-7012)
		(trans room-7011 room-7015)

 		 ;;; transitions for room-5003
		(trans room-5003 room-5002)
		(trans room-5003 room-5004)

 		 ;;; transitions for room-7009
		(trans room-7009 room-7011)
		(trans room-7009 room-7013)
		(trans room-7009 room-7014)

 		 ;;; transitions for room-5002
		(trans room-5002 room-5003)
		(trans room-5002 room-5004)

 		 ;;; transitions for room-1003
		(trans room-1003 room-1002)
		(trans room-1003 room-1006)

 		 ;;; transitions for room-1002
		(trans room-1002 room-1003)
		(trans room-1002 room-1001)
		(trans room-1002 room-1005)

 		 ;;; transitions for room-1001
		(trans room-1001 room-1002)
		(trans room-1001 room-1004)

 		 ;;; transitions for room-7012
		(trans room-7012 room-7011)
		(trans room-7012 room-7017)
		(trans room-7012 room-7016)

 		 ;;; transitions for room-7017
		(trans room-7017 room-7012)
		(trans room-7017 room-7016)
		(trans room-7017 room-7023)

 		 ;;; transitions for room-1004
		(trans room-1004 room-1005)
		(trans room-1004 room-1001)

 		 ;;; transitions for room-1005
		(trans room-1005 room-1004)
		(trans room-1005 room-1006)
		(trans room-1005 room-1002)

 		 ;;; transitions for room-1006
		(trans room-1006 room-1005)
		(trans room-1006 room-1003)
		(trans room-1006 room-2001)
		(trans room-1006 room-2004)

 		 ;;; transitions for room-5004
		(trans room-5004 room-5003)
		(trans room-5004 room-2002)
		(trans room-5004 room-2005)
		(trans room-5004 room-5002)

 		 ;;; transitions for room-7013
		(trans room-7013 room-7014)
		(trans room-7013 room-7009)
		(trans room-7013 room-7018)
		(trans room-7013 room-7019)

 		 ;;; transitions for room-7014
		(trans room-7014 room-7013)
		(trans room-7014 room-7015)
		(trans room-7014 room-7009)
		(trans room-7014 room-7020)

 		 ;;; transitions for room-7015
		(trans room-7015 room-7014)
		(trans room-7015 room-7016)
		(trans room-7015 room-7011)
		(trans room-7015 room-7021)

 		 ;;; transitions for room-7016
		(trans room-7016 room-7015)
		(trans room-7016 room-7017)
		(trans room-7016 room-7012)
		(trans room-7016 room-7022)

 		 ;;; transitions for room-6001
		(trans room-6001 room-2002)
		(trans room-6001 room-6004)
		(trans room-6001 room-6003)
		(trans room-6001 room-6002)

 		 ;;; transitions for room-7018
		(trans room-7018 room-7013)
		(trans room-7018 room-6004)
		(trans room-7018 room-7019)

 		 ;;; transitions for room-2002
		(trans room-2002 room-5004)
		(trans room-2002 room-2005)
		(trans room-2002 room-6001)
		(trans room-2002 room-6002)
		(trans room-2002 room-2006)

 		 ;;; transitions for room-2001
		(trans room-2001 room-1006)
		(trans room-2001 room-2004)

 		 ;;; transitions for room-2005
		(trans room-2005 room-2002)
		(trans room-2005 room-5004)
		(trans room-2005 room-2004)
		(trans room-2005 room-2006)
		(trans room-2005 room-4001)
		(trans room-2005 room-4003)

 		 ;;; transitions for room-2004
		(trans room-2004 room-1006)
		(trans room-2004 room-2001)
		(trans room-2004 room-2005)
		(trans room-2004 room-2003)

 		 ;;; transitions for room-2003
		(trans room-2003 room-2004)
		(trans room-2003 room-3001)

 		 ;;; transitions for room-6004
		(trans room-6004 room-6001)
		(trans room-6004 room-7018)
		(trans room-6004 room-6003)

 		 ;;; transitions for room-2006
		(trans room-2006 room-2005)
		(trans room-2006 room-6002)
		(trans room-2006 room-2002)
		(trans room-2006 room-4004)
		(trans room-2006 room-2007)

 		 ;;; transitions for room-6002
		(trans room-6002 room-2006)
		(trans room-6002 room-6003)
		(trans room-6002 room-6001)
		(trans room-6002 room-6005)
		(trans room-6002 room-2002)

 		 ;;; transitions for room-6003
		(trans room-6003 room-6002)
		(trans room-6003 room-6004)
		(trans room-6003 room-6001)
		(trans room-6003 room-6005)

 		 ;;; transitions for room-7019
		(trans room-7019 room-7018)
		(trans room-7019 room-7020)
		(trans room-7019 room-7013)
		(trans room-7019 room-7024)

 		 ;;; transitions for room-7020
		(trans room-7020 room-7019)
		(trans room-7020 room-7021)
		(trans room-7020 room-7014)
		(trans room-7020 room-7024)

 		 ;;; transitions for room-7021
		(trans room-7021 room-7020)
		(trans room-7021 room-7022)
		(trans room-7021 room-7015)
		(trans room-7021 room-7025)

 		 ;;; transitions for room-7022
		(trans room-7022 room-7021)
		(trans room-7022 room-7023)
		(trans room-7022 room-7016)
		(trans room-7022 room-7026)

 		 ;;; transitions for room-7023
		(trans room-7023 room-7022)
		(trans room-7023 room-7017)
		(trans room-7023 room-7026)

 		 ;;; transitions for room-4001
		(trans room-4001 room-2005)
		(trans room-4001 room-4003)

 		 ;;; transitions for room-4002
		(trans room-4002 room-4003)
		(trans room-4002 room-4005)

 		 ;;; transitions for room-4003
		(trans room-4003 room-4002)
		(trans room-4003 room-4004)
		(trans room-4003 room-2005)
		(trans room-4003 room-4006)
		(trans room-4003 room-4001)

 		 ;;; transitions for room-4004
		(trans room-4004 room-4003)
		(trans room-4004 room-2007)
		(trans room-4004 room-2006)
		(trans room-4004 room-5001)

 		 ;;; transitions for room-2007
		(trans room-2007 room-4004)
		(trans room-2007 room-2006)

 		 ;;; transitions for room-6005
		(trans room-6005 room-6003)
		(trans room-6005 room-7001)
		(trans room-6005 room-6002)

 		 ;;; transitions for room-7024
		(trans room-7024 room-7019)
		(trans room-7024 room-7025)
		(trans room-7024 room-7020)
		(trans room-7024 room-8001)

 		 ;;; transitions for room-7025
		(trans room-7025 room-7024)
		(trans room-7025 room-7026)
		(trans room-7025 room-7021)
		(trans room-7025 room-8001)

 		 ;;; transitions for room-7026
		(trans room-7026 room-7025)
		(trans room-7026 room-7023)
		(trans room-7026 room-8001)
		(trans room-7026 room-7022)

 		 ;;; transitions for room-3001
		(trans room-3001 room-2003)
		(trans room-3001 room-3002)

 		 ;;; transitions for room-3002
		(trans room-3002 room-3001)

 		 ;;; transitions for room-4005
		(trans room-4005 room-4006)
		(trans room-4005 room-4002)

 		 ;;; transitions for room-4006
		(trans room-4006 room-4005)
		(trans room-4006 room-5001)
		(trans room-4006 room-4003)

 		 ;;; transitions for room-5001
		(trans room-5001 room-4006)
		(trans room-5001 room-4004)

 		 ;;; transitions for room-7001
		(trans room-7001 room-7004)
		(trans room-7001 room-7003)
		(trans room-7001 room-6005)

 		 ;;; transitions for room-8001
		(trans room-8001 room-7024)
		(trans room-8001 room-7026)
		(trans room-8001 room-7002)
		(trans room-8001 room-7025)

 		 ;;; transitions for room-7004
		(trans room-7004 room-7001)
		(trans room-7004 room-7003)
		(trans room-7004 room-7005)
		(trans room-7004 room-7006)

 		 ;;; transitions for room-7003
		(trans room-7003 room-7004)
		(trans room-7003 room-7001)
		(trans room-7003 room-7006)

 		 ;;; transitions for room-7005
		(trans room-7005 room-7004)
		(trans room-7005 room-7008)
		(trans room-7005 room-7007)

 		 ;;; transitions for room-7002
		(trans room-7002 room-8001)
		(trans room-7002 room-7010)

 		 ;;; transitions for room-7006
		(trans room-7006 room-7003)
		(trans room-7006 room-7007)
		(trans room-7006 room-7004)

 		 ;;; transitions for room-7007
		(trans room-7007 room-7006)
		(trans room-7007 room-7008)
		(trans room-7007 room-7005)

 		 ;;; transitions for room-7008
		(trans room-7008 room-7007)
		(trans room-7008 room-7010)
		(trans room-7008 room-7005)

 		 ;;; transitions for room-7010
		(trans room-7010 room-7002)
		(trans room-7010 room-7008)
		(trans room-7010 room-9001)

 		 ;;; transitions for room-10001
		(trans room-10001 room-10002)
		(trans room-10001 room-10004)

 		 ;;; transitions for room-10002
		(trans room-10002 room-10001)
		(trans room-10002 room-9001)
		(trans room-10002 room-10003)

 		 ;;; transitions for room-9001
		(trans room-9001 room-9002)
		(trans room-9001 room-10002)
		(trans room-9001 room-9006)
		(trans room-9001 room-7010)

 		 ;;; transitions for room-9002
		(trans room-9002 room-9001)
		(trans room-9002 room-9003)
		(trans room-9002 room-9005)

 		 ;;; transitions for room-9003
		(trans room-9003 room-9002)
		(trans room-9003 room-9004)

 		 ;;; transitions for room-9004
		(trans room-9004 room-9005)
		(trans room-9004 room-9003)
		(trans room-9004 room-9007)

 		 ;;; transitions for room-9005
		(trans room-9005 room-9004)
		(trans room-9005 room-9006)
		(trans room-9005 room-9002)
		(trans room-9005 room-9008)

 		 ;;; transitions for room-9006
		(trans room-9006 room-9005)
		(trans room-9006 room-10003)
		(trans room-9006 room-9001)
		(trans room-9006 room-9009)

 		 ;;; transitions for room-10003
		(trans room-10003 room-9006)
		(trans room-10003 room-10004)
		(trans room-10003 room-10002)
		(trans room-10003 room-10005)

 		 ;;; transitions for room-10004
		(trans room-10004 room-10003)
		(trans room-10004 room-10001)
		(trans room-10004 room-10006)

 		 ;;; transitions for room-9007
		(trans room-9007 room-9008)
		(trans room-9007 room-9004)
		(trans room-9007 room-9010)

 		 ;;; transitions for room-9008
		(trans room-9008 room-9007)
		(trans room-9008 room-9009)
		(trans room-9008 room-9005)
		(trans room-9008 room-9011)

 		 ;;; transitions for room-9009
		(trans room-9009 room-9008)
		(trans room-9009 room-9006)
		(trans room-9009 room-10005)
		(trans room-9009 room-9012)

 		 ;;; transitions for room-10005
		(trans room-10005 room-10006)
		(trans room-10005 room-10003)
		(trans room-10005 room-9009)
		(trans room-10005 room-10007)

 		 ;;; transitions for room-10006
		(trans room-10006 room-10005)
		(trans room-10006 room-10004)
		(trans room-10006 room-10008)

 		 ;;; transitions for room-9010
		(trans room-9010 room-9011)
		(trans room-9010 room-9007)

 		 ;;; transitions for room-9011
		(trans room-9011 room-9010)
		(trans room-9011 room-9012)
		(trans room-9011 room-9008)

 		 ;;; transitions for room-9012
		(trans room-9012 room-9011)
		(trans room-9012 room-10007)
		(trans room-9012 room-9009)

 		 ;;; transitions for room-10007
		(trans room-10007 room-9012)
		(trans room-10007 room-10008)
		(trans room-10007 room-10005)

 		 ;;; transitions for room-10008
		(trans room-10008 room-10007)
		(trans room-10008 room-10006)

	 ;;; hard coded definitions
		(is-robo cob4-1)
		(at the-cake room-9007)
		(at cob4-1 room-7007)
	)


	;;; goal definition
	(:goal (and (have cob4-1 the-cake) (at cob4-1 room-7007)))
)