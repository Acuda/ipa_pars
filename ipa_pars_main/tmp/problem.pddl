(define (problem cob-test-problem-01)
 	(:domain cob-test-domain-01)
 	(:objects 
  		 ;;; available objects 
  
  		 ;;; fixed locations 
 		 room-9-square-3 room-9-square-2 room-9-square-1 room-9-square-4 
 		 room-5-square-3 room-5-square-2 room-5-square-1 room-1-square-3 
 		 room-1-square-2 room-9-square-5 room-9-square-6 room-9-square-7 
 		 room-9-square-8 room-9-square-9 room-1-square-1 room-1-square-4 
 		 room-1-square-5 room-9-square-16 room-1-square-6 room-1-square-7 
 		 room-1-square-8 room-1-square-9 room-1-square-10 room-5-square-4 
 		 room-5-square-5 room-5-square-6 room-9-square-10 room-9-square-11 
 		 room-9-square-12 room-9-square-13 room-9-square-14 room-9-square-15 
 		 room-9-square-25 room-1-square-11 room-1-square-12 room-1-square-13 
 		 room-1-square-14 room-1-square-15 room-5-square-7 room-5-square-8 
 		 room-5-square-9 room-9-square-17 room-9-square-18 room-9-square-19 
 		 room-9-square-20 room-9-square-21 room-9-square-22 room-9-square-23 
 		 room-9-square-24 room-6-square-1 room-6-square-3 room-1-square-16 
 		 room-1-square-17 room-1-square-18 room-1-square-19 room-1-square-20 
 		 room-5-square-10 room-5-square-11 room-5-square-12 room-6-square-4 
 		 room-6-square-5 room-9-square-26 room-9-square-27 room-9-square-28 
 		 room-9-square-29 room-9-square-30 room-9-square-31 room-9-square-32 
 		 room-9-square-33 room-9-square-34 room-9-square-35 room-6-square-2 
 		 room-6-square-11 room-6-square-10 room-1-square-21 room-1-square-22 
 		 room-1-square-23 room-5-square-13 room-6-square-6 room-6-square-7 
 		 room-6-square-8 room-6-square-9 room-9-square-36 room-9-square-37 
 		 room-9-square-38 room-9-square-39 room-9-square-40 room-9-square-41 
 		 room-9-square-42 room-9-square-43 room-9-square-44 room-9-square-45 
 		 room-9-square-46 room-9-square-47 room-2-square-2 room-2-square-1 
 		 room-2-square-3 room-2-square-4 room-6-square-12 room-2-square-8 
 		 room-2-square-7 room-2-square-6 room-2-square-5 room-2-square-9 
 		 room-2-square-10 room-2-square-11 room-6-square-13 room-6-square-14 
 		 room-6-square-15 room-6-square-16 room-6-square-17 room-6-square-18 
 		 room-9-square-48 room-9-square-49 room-9-square-50 room-9-square-51 
 		 room-9-square-52 room-9-square-53 room-9-square-54 room-9-square-55 
 		 room-9-square-56 room-9-square-57 room-7-square-3 room-7-square-2 
 		 room-7-square-1 room-2-square-12 room-2-square-13 room-6-square-19 
 		 room-7-square-4 room-7-square-5 room-9-square-58 room-9-square-59 
 		 room-9-square-60 room-9-square-61 room-9-square-62 room-9-square-63 
 		 room-9-square-64 room-9-square-65 room-4-square-1 room-4-square-2 
 		 room-3-square-2 room-4-square-3 room-4-square-4 room-4-square-5 
 		 room-3-square-1 room-3-square-4 room-3-square-3 room-3-square-5 
 		 room-4-square-6 room-4-square-7 room-4-square-8 room-4-square-9 
 		 room-4-square-10 room-7-square-6 room-7-square-7 room-9-square-66 
 		 room-9-square-67 room-9-square-68 room-9-square-69 room-9-square-70 
 		 room-9-square-71 room-8-square-3 room-13-square-1 room-13-square-2 
 		 room-8-square-1 room-8-square-2 room-3-square-6 room-3-square-7 
 		 room-4-square-11 room-4-square-12 room-4-square-13 room-4-square-14 
 		 room-4-square-15 room-4-square-16 room-8-square-4 room-9-square-72 
 		 room-9-square-73 room-9-square-74 room-13-square-3 room-8-square-8 
 		 room-3-square-8 room-3-square-9 room-4-square-17 room-4-square-18 
 		 room-4-square-19 room-4-square-20 room-4-square-21 room-4-square-22 
 		 room-8-square-5 room-8-square-6 room-8-square-7 room-9-square-75 
 		 room-9-square-76 room-8-square-13 room-3-square-10 room-3-square-11 
 		 room-4-square-23 room-4-square-24 room-4-square-25 room-4-square-26 
 		 room-4-square-27 room-8-square-9 room-8-square-10 room-8-square-11 
 		 room-8-square-12 room-9-square-77 room-9-square-78 room-8-square-18 
 		 room-12-square-1 room-2-square-14 room-8-square-14 room-8-square-15 
 		 room-8-square-16 room-8-square-17 room-9-square-79 room-12-square-2 
 		 room-12-square-3 room-8-square-19 room-12-square-10 room-12-square-7 
 		 room-12-square-6 room-12-square-4 room-8-square-20 room-8-square-21 
 		 room-8-square-22 room-8-square-23 room-8-square-24 room-8-square-25 
 		 room-12-square-8 room-12-square-9 room-12-square-5 room-11-square-3 
 		 room-11-square-1 room-11-square-2 room-8-square-26 room-11-square-4 
 		 room-12-square-11 room-12-square-12 room-12-square-13 room-12-square-14 
 		 room-10-square-8 room-10-square-7 room-10-square-6 room-10-square-5 
 		 room-10-square-4 room-10-square-3 room-10-square-2 room-10-square-1 
 		 room-10-square-9 room-10-square-12 room-9-square-80 room-10-square-11 
 		 room-10-square-10 room-10-square-13 room-10-square-14 room-10-square-15 
 		 room-10-square-16 room-10-square-17 room-10-square-18 room-10-square-19 
 		 room-10-square-20 room-10-square-21 room-10-square-22 room-10-square-23 
 		 room-10-square-24 room-10-square-25 room-10-square-26 room-10-square-27 
 		 room-10-square-28 room-10-square-29 room-10-square-30 room-10-square-31 
 		 room-10-square-32 room-10-square-33 room-10-square-34 room-10-square-35 
 		 room-10-square-36 room-10-square-37 room-10-square-38 room-10-square-39 
 		 room-10-square-40 room-10-square-41 room-10-square-42 room-10-square-43 
 		 room-10-square-44 room-10-square-45 room-10-square-46 room-10-square-47 
 		 room-10-square-48 room-10-square-49 room-10-square-50 room-10-square-51 
 		 room-10-square-52 room-10-square-53 room-10-square-54 room-10-square-55 
 		 room-10-square-56 room-10-square-57 room-10-square-58 room-10-square-59 
 		 room-10-square-60 room-10-square-61 room-10-square-62 room-10-square-63 
 		 room-10-square-64 room-10-square-65 room-10-square-66 room-10-square-67 
 		 room-10-square-68 room-10-square-69 room-10-square-70 room-10-square-71 
 		 room-10-square-72 room-10-square-73 room-10-square-74 room-10-square-75 
 		 room-10-square-76 room-10-square-77 room-10-square-78 room-10-square-79 
 		 room-10-square-80 room-10-square-81 room-10-square-82 room-10-square-83 
 		 room-10-square-84 room-10-square-85 room-10-square-86 room-10-square-87 
  
 		 ;;; fixed things for interaction
 
 		 user
 		 door-1
  		 ;;; movable things 
  		
 		 the-cake 
 		 cob4-1
 		the-box
 		)
 
 
 	(:init  
  		 ;;; transitions for room-9-square-3
 		(trans room-9-square-3 room-9-square-2)
 		(trans room-9-square-3 room-9-square-4)
 		(trans room-9-square-3 room-9-square-7)
 
  		 ;;; transitions for room-9-square-2
 		(trans room-9-square-2 room-9-square-3)
 		(trans room-9-square-2 room-9-square-1)
 		(trans room-9-square-2 room-9-square-6)
 
  		 ;;; transitions for room-9-square-1
 		(trans room-9-square-1 room-9-square-2)
 		(trans room-9-square-1 room-9-square-5)
 
  		 ;;; transitions for room-9-square-4
 		(trans room-9-square-4 room-9-square-3)
 		(trans room-9-square-4 room-9-square-9)
 		(trans room-9-square-4 room-9-square-8)
 
  		 ;;; transitions for room-5-square-3
 		(trans room-5-square-3 room-5-square-2)
 		(trans room-5-square-3 room-5-square-6)
 
  		 ;;; transitions for room-5-square-2
 		(trans room-5-square-2 room-5-square-3)
 		(trans room-5-square-2 room-5-square-1)
 		(trans room-5-square-2 room-5-square-5)
 
  		 ;;; transitions for room-5-square-1
 		(trans room-5-square-1 room-5-square-2)
 		(trans room-5-square-1 room-5-square-4)
 
  		 ;;; transitions for room-1-square-3
 		(trans room-1-square-3 room-1-square-2)
 		(trans room-1-square-3 room-1-square-10)
 
  		 ;;; transitions for room-1-square-2
 		(trans room-1-square-2 room-1-square-3)
 		(trans room-1-square-2 room-1-square-1)
 		(trans room-1-square-2 room-1-square-9)
 
  		 ;;; transitions for room-9-square-5
 		(trans room-9-square-5 room-9-square-6)
 		(trans room-9-square-5 room-9-square-1)
 		(trans room-9-square-5 room-9-square-10)
 		(trans room-9-square-5 room-9-square-11)
 
  		 ;;; transitions for room-9-square-6
 		(trans room-9-square-6 room-9-square-5)
 		(trans room-9-square-6 room-9-square-7)
 		(trans room-9-square-6 room-9-square-2)
 		(trans room-9-square-6 room-9-square-12)
 
  		 ;;; transitions for room-9-square-7
 		(trans room-9-square-7 room-9-square-6)
 		(trans room-9-square-7 room-9-square-8)
 		(trans room-9-square-7 room-9-square-3)
 		(trans room-9-square-7 room-9-square-13)
 
  		 ;;; transitions for room-9-square-8
 		(trans room-9-square-8 room-9-square-7)
 		(trans room-9-square-8 room-9-square-9)
 		(trans room-9-square-8 room-9-square-4)
 		(trans room-9-square-8 room-9-square-14)
 
  		 ;;; transitions for room-9-square-9
 		(trans room-9-square-9 room-9-square-8)
 		(trans room-9-square-9 room-9-square-16)
 		(trans room-9-square-9 room-9-square-15)
 		(trans room-9-square-9 room-9-square-4)
 
  		 ;;; transitions for room-1-square-1
 		(trans room-1-square-1 room-1-square-2)
 		(trans room-1-square-1 room-1-square-4)
 		(trans room-1-square-1 room-1-square-8)
 
  		 ;;; transitions for room-1-square-4
 		(trans room-1-square-4 room-1-square-1)
 		(trans room-1-square-4 room-1-square-5)
 		(trans room-1-square-4 room-1-square-7)
 
  		 ;;; transitions for room-1-square-5
 		(trans room-1-square-5 room-1-square-4)
 		(trans room-1-square-5 room-1-square-6)
 
  		 ;;; transitions for room-9-square-16
 		(trans room-9-square-16 room-9-square-9)
 		(trans room-9-square-16 room-9-square-15)
 		(trans room-9-square-16 room-9-square-25)
 		(trans room-9-square-16 room-9-square-24)
 
  		 ;;; transitions for room-1-square-6
 		(trans room-1-square-6 room-1-square-7)
 		(trans room-1-square-6 room-1-square-5)
 		(trans room-1-square-6 room-1-square-11)
 
  		 ;;; transitions for room-1-square-7
 		(trans room-1-square-7 room-1-square-6)
 		(trans room-1-square-7 room-1-square-8)
 		(trans room-1-square-7 room-1-square-4)
 		(trans room-1-square-7 room-1-square-12)
 
  		 ;;; transitions for room-1-square-8
 		(trans room-1-square-8 room-1-square-7)
 		(trans room-1-square-8 room-1-square-9)
 		(trans room-1-square-8 room-1-square-1)
 		(trans room-1-square-8 room-1-square-13)
 
  		 ;;; transitions for room-1-square-9
 		(trans room-1-square-9 room-1-square-8)
 		(trans room-1-square-9 room-1-square-10)
 		(trans room-1-square-9 room-1-square-2)
 		(trans room-1-square-9 room-1-square-14)
 
  		 ;;; transitions for room-1-square-10
 		(trans room-1-square-10 room-1-square-9)
 		(trans room-1-square-10 room-1-square-3)
 		(trans room-1-square-10 room-1-square-15)
 
  		 ;;; transitions for room-5-square-4
 		(trans room-5-square-4 room-5-square-5)
 		(trans room-5-square-4 room-5-square-1)
 		(trans room-5-square-4 room-5-square-7)
 
  		 ;;; transitions for room-5-square-5
 		(trans room-5-square-5 room-5-square-4)
 		(trans room-5-square-5 room-5-square-6)
 		(trans room-5-square-5 room-5-square-2)
 		(trans room-5-square-5 room-5-square-8)
 
  		 ;;; transitions for room-5-square-6
 		(trans room-5-square-6 room-5-square-5)
 		(trans room-5-square-6 room-5-square-3)
 		(trans room-5-square-6 room-5-square-9)
 
  		 ;;; transitions for room-9-square-10
 		(trans room-9-square-10 room-9-square-11)
 		(trans room-9-square-10 room-9-square-5)
 		(trans room-9-square-10 room-9-square-17)
 		(trans room-9-square-10 room-9-square-18)
 
  		 ;;; transitions for room-9-square-11
 		(trans room-9-square-11 room-9-square-10)
 		(trans room-9-square-11 room-9-square-12)
 		(trans room-9-square-11 room-9-square-5)
 		(trans room-9-square-11 room-9-square-19)
 
  		 ;;; transitions for room-9-square-12
 		(trans room-9-square-12 room-9-square-11)
 		(trans room-9-square-12 room-9-square-13)
 		(trans room-9-square-12 room-9-square-6)
 		(trans room-9-square-12 room-9-square-20)
 
  		 ;;; transitions for room-9-square-13
 		(trans room-9-square-13 room-9-square-12)
 		(trans room-9-square-13 room-9-square-14)
 		(trans room-9-square-13 room-9-square-7)
 		(trans room-9-square-13 room-9-square-21)
 
  		 ;;; transitions for room-9-square-14
 		(trans room-9-square-14 room-9-square-13)
 		(trans room-9-square-14 room-9-square-15)
 		(trans room-9-square-14 room-9-square-8)
 		(trans room-9-square-14 room-9-square-22)
 
  		 ;;; transitions for room-9-square-15
 		(trans room-9-square-15 room-9-square-14)
 		(trans room-9-square-15 room-9-square-16)
 		(trans room-9-square-15 room-9-square-9)
 		(trans room-9-square-15 room-9-square-23)
 
  		 ;;; transitions for room-9-square-25
 		(trans room-9-square-25 room-9-square-16)
 		(trans room-9-square-25 room-9-square-24)
 		(trans room-9-square-25 room-9-square-35)
 
  		 ;;; transitions for room-1-square-11
 		(trans room-1-square-11 room-1-square-12)
 		(trans room-1-square-11 room-1-square-6)
 		(trans room-1-square-11 room-1-square-16)
 
  		 ;;; transitions for room-1-square-12
 		(trans room-1-square-12 room-1-square-11)
 		(trans room-1-square-12 room-1-square-13)
 		(trans room-1-square-12 room-1-square-7)
 		(trans room-1-square-12 room-1-square-17)
 
  		 ;;; transitions for room-1-square-13
 		(trans room-1-square-13 room-1-square-12)
 		(trans room-1-square-13 room-1-square-14)
 		(trans room-1-square-13 room-1-square-8)
 		(trans room-1-square-13 room-1-square-18)
 
  		 ;;; transitions for room-1-square-14
 		(trans room-1-square-14 room-1-square-13)
 		(trans room-1-square-14 room-1-square-15)
 		(trans room-1-square-14 room-1-square-9)
 		(trans room-1-square-14 room-1-square-19)
 
  		 ;;; transitions for room-1-square-15
 		(trans room-1-square-15 room-1-square-14)
 		(trans room-1-square-15 room-1-square-10)
 		(trans room-1-square-15 room-1-square-20)
 
  		 ;;; transitions for room-5-square-7
 		(trans room-5-square-7 room-5-square-8)
 		(trans room-5-square-7 room-5-square-4)
 		(trans room-5-square-7 room-5-square-10)
 
  		 ;;; transitions for room-5-square-8
 		(trans room-5-square-8 room-5-square-7)
 		(trans room-5-square-8 room-5-square-9)
 		(trans room-5-square-8 room-5-square-5)
 		(trans room-5-square-8 room-5-square-11)
 
  		 ;;; transitions for room-5-square-9
 		(trans room-5-square-9 room-5-square-8)
 		(trans room-5-square-9 room-5-square-6)
 		(trans room-5-square-9 room-5-square-12)
 
  		 ;;; transitions for room-9-square-17
 		(trans room-9-square-17 room-9-square-18)
 		(trans room-9-square-17 room-9-square-10)
 		(trans room-9-square-17 room-9-square-26)
 		(trans room-9-square-17 room-9-square-27)
 
  		 ;;; transitions for room-9-square-18
 		(trans room-9-square-18 room-9-square-17)
 		(trans room-9-square-18 room-9-square-19)
 		(trans room-9-square-18 room-9-square-10)
 		(trans room-9-square-18 room-9-square-28)
 
  		 ;;; transitions for room-9-square-19
 		(trans room-9-square-19 room-9-square-18)
 		(trans room-9-square-19 room-9-square-20)
 		(trans room-9-square-19 room-9-square-11)
 		(trans room-9-square-19 room-9-square-29)
 
  		 ;;; transitions for room-9-square-20
 		(trans room-9-square-20 room-9-square-19)
 		(trans room-9-square-20 room-9-square-21)
 		(trans room-9-square-20 room-9-square-12)
 		(trans room-9-square-20 room-9-square-30)
 
  		 ;;; transitions for room-9-square-21
 		(trans room-9-square-21 room-9-square-20)
 		(trans room-9-square-21 room-9-square-22)
 		(trans room-9-square-21 room-9-square-13)
 		(trans room-9-square-21 room-9-square-31)
 
  		 ;;; transitions for room-9-square-22
 		(trans room-9-square-22 room-9-square-21)
 		(trans room-9-square-22 room-9-square-23)
 		(trans room-9-square-22 room-9-square-14)
 		(trans room-9-square-22 room-9-square-32)
 
  		 ;;; transitions for room-9-square-23
 		(trans room-9-square-23 room-9-square-22)
 		(trans room-9-square-23 room-9-square-24)
 		(trans room-9-square-23 room-9-square-15)
 		(trans room-9-square-23 room-9-square-33)
 
  		 ;;; transitions for room-9-square-24
 		(trans room-9-square-24 room-9-square-23)
 		(trans room-9-square-24 room-9-square-25)
 		(trans room-9-square-24 room-9-square-16)
 		(trans room-9-square-24 room-9-square-34)
 
  		 ;;; transitions for room-6-square-1
 		(trans room-6-square-1 room-6-square-3)
 		(trans room-6-square-1 room-6-square-4)
 		(trans room-6-square-1 room-6-square-5)
 
  		 ;;; transitions for room-6-square-3
 		(trans room-6-square-3 room-6-square-1)
 		(trans room-6-square-3 room-6-square-4)
 		(trans room-6-square-3 room-6-square-7)
 
  		 ;;; transitions for room-1-square-16
 		(trans room-1-square-16 room-1-square-17)
 		(trans room-1-square-16 room-1-square-11)
 		(trans room-1-square-16 room-1-square-21)
 
  		 ;;; transitions for room-1-square-17
 		(trans room-1-square-17 room-1-square-16)
 		(trans room-1-square-17 room-1-square-18)
 		(trans room-1-square-17 room-1-square-12)
 		(trans room-1-square-17 room-1-square-22)
 
  		 ;;; transitions for room-1-square-18
 		(trans room-1-square-18 room-1-square-17)
 		(trans room-1-square-18 room-1-square-19)
 		(trans room-1-square-18 room-1-square-13)
 		(trans room-1-square-18 room-1-square-23)
 
  		 ;;; transitions for room-1-square-19
 		(trans room-1-square-19 room-1-square-18)
 		(trans room-1-square-19 room-1-square-20)
 		(trans room-1-square-19 room-1-square-14)
 		(trans room-1-square-19 room-1-square-23)
 
  		 ;;; transitions for room-1-square-20
 		(trans room-1-square-20 room-1-square-19)
 		(trans room-1-square-20 room-2-square-2)
 		(trans room-1-square-20 room-2-square-1)
 		(trans room-1-square-20 room-2-square-3)
 		(trans room-1-square-20 room-1-square-15)
 
  		 ;;; transitions for room-5-square-10
 		(trans room-5-square-10 room-5-square-11)
 		(trans room-5-square-10 room-5-square-7)
 
  		 ;;; transitions for room-5-square-11
 		(trans room-5-square-11 room-5-square-10)
 		(trans room-5-square-11 room-5-square-12)
 		(trans room-5-square-11 room-5-square-8)
 
  		 ;;; transitions for room-5-square-12
 		(trans room-5-square-12 room-5-square-11)
 		(trans room-5-square-12 room-5-square-9)
 		(trans room-5-square-12 room-6-square-2)
 		(trans room-5-square-12 room-5-square-13)
 		(trans room-5-square-12 room-6-square-6)
 
  		 ;;; transitions for room-6-square-4
 		(trans room-6-square-4 room-6-square-3)
 		(trans room-6-square-4 room-6-square-5)
 		(trans room-6-square-4 room-6-square-1)
 		(trans room-6-square-4 room-6-square-8)
 
  		 ;;; transitions for room-6-square-5
 		(trans room-6-square-5 room-6-square-4)
 		(trans room-6-square-5 room-6-square-1)
 		(trans room-6-square-5 room-6-square-9)
 
  		 ;;; transitions for room-9-square-26
 		(trans room-9-square-26 room-9-square-27)
 		(trans room-9-square-26 room-9-square-17)
 		(trans room-9-square-26 room-6-square-11)
 		(trans room-9-square-26 room-9-square-37)
 		(trans room-9-square-26 room-9-square-36)
 
  		 ;;; transitions for room-9-square-27
 		(trans room-9-square-27 room-9-square-26)
 		(trans room-9-square-27 room-9-square-28)
 		(trans room-9-square-27 room-9-square-17)
 		(trans room-9-square-27 room-9-square-38)
 
  		 ;;; transitions for room-9-square-28
 		(trans room-9-square-28 room-9-square-27)
 		(trans room-9-square-28 room-9-square-29)
 		(trans room-9-square-28 room-9-square-18)
 		(trans room-9-square-28 room-9-square-39)
 
  		 ;;; transitions for room-9-square-29
 		(trans room-9-square-29 room-9-square-28)
 		(trans room-9-square-29 room-9-square-30)
 		(trans room-9-square-29 room-9-square-19)
 		(trans room-9-square-29 room-9-square-40)
 
  		 ;;; transitions for room-9-square-30
 		(trans room-9-square-30 room-9-square-29)
 		(trans room-9-square-30 room-9-square-31)
 		(trans room-9-square-30 room-9-square-20)
 		(trans room-9-square-30 room-9-square-41)
 
  		 ;;; transitions for room-9-square-31
 		(trans room-9-square-31 room-9-square-30)
 		(trans room-9-square-31 room-9-square-32)
 		(trans room-9-square-31 room-9-square-21)
 		(trans room-9-square-31 room-9-square-42)
 
  		 ;;; transitions for room-9-square-32
 		(trans room-9-square-32 room-9-square-31)
 		(trans room-9-square-32 room-9-square-33)
 		(trans room-9-square-32 room-9-square-22)
 		(trans room-9-square-32 room-9-square-43)
 
  		 ;;; transitions for room-9-square-33
 		(trans room-9-square-33 room-9-square-32)
 		(trans room-9-square-33 room-9-square-34)
 		(trans room-9-square-33 room-9-square-23)
 		(trans room-9-square-33 room-9-square-44)
 
  		 ;;; transitions for room-9-square-34
 		(trans room-9-square-34 room-9-square-33)
 		(trans room-9-square-34 room-9-square-35)
 		(trans room-9-square-34 room-9-square-24)
 		(trans room-9-square-34 room-9-square-45)
 
  		 ;;; transitions for room-9-square-35
 		(trans room-9-square-35 room-9-square-34)
 		(trans room-9-square-35 room-9-square-25)
 		(trans room-9-square-35 room-9-square-46)
 		(trans room-9-square-35 room-9-square-47)
 
  		 ;;; transitions for room-6-square-2
 		(trans room-6-square-2 room-5-square-12)
 		(trans room-6-square-2 room-6-square-6)
 		(trans room-6-square-2 room-6-square-7)
 
  		 ;;; transitions for room-6-square-11
 		(trans room-6-square-11 room-6-square-10)
 		(trans room-6-square-11 room-9-square-36)
 		(trans room-6-square-11 room-9-square-26)
 		(trans room-6-square-11 room-6-square-18)
 
  		 ;;; transitions for room-6-square-10
 		(trans room-6-square-10 room-6-square-11)
 		(trans room-6-square-10 room-6-square-9)
 		(trans room-6-square-10 room-6-square-18)
 
  		 ;;; transitions for room-1-square-21
 		(trans room-1-square-21 room-1-square-22)
 		(trans room-1-square-21 room-1-square-16)
 
  		 ;;; transitions for room-1-square-22
 		(trans room-1-square-22 room-1-square-21)
 		(trans room-1-square-22 room-1-square-23)
 		(trans room-1-square-22 room-1-square-17)
 
  		 ;;; transitions for room-1-square-23
 		(trans room-1-square-23 room-1-square-22)
 		(trans room-1-square-23 room-1-square-19)
 		(trans room-1-square-23 room-1-square-18)
 
  		 ;;; transitions for room-5-square-13
 		(trans room-5-square-13 room-5-square-12)
 		(trans room-5-square-13 room-6-square-6)
 
  		 ;;; transitions for room-6-square-6
 		(trans room-6-square-6 room-5-square-13)
 		(trans room-6-square-6 room-6-square-7)
 		(trans room-6-square-6 room-6-square-2)
 		(trans room-6-square-6 room-6-square-12)
 		(trans room-6-square-6 room-6-square-14)
 		(trans room-6-square-6 room-5-square-12)
 
  		 ;;; transitions for room-6-square-7
 		(trans room-6-square-7 room-6-square-6)
 		(trans room-6-square-7 room-6-square-2)
 		(trans room-6-square-7 room-6-square-8)
 		(trans room-6-square-7 room-6-square-3)
 		(trans room-6-square-7 room-6-square-15)
 
  		 ;;; transitions for room-6-square-8
 		(trans room-6-square-8 room-6-square-7)
 		(trans room-6-square-8 room-6-square-9)
 		(trans room-6-square-8 room-6-square-4)
 		(trans room-6-square-8 room-6-square-16)
 
  		 ;;; transitions for room-6-square-9
 		(trans room-6-square-9 room-6-square-8)
 		(trans room-6-square-9 room-6-square-5)
 		(trans room-6-square-9 room-6-square-10)
 		(trans room-6-square-9 room-6-square-17)
 
  		 ;;; transitions for room-9-square-36
 		(trans room-9-square-36 room-6-square-11)
 		(trans room-9-square-36 room-9-square-37)
 		(trans room-9-square-36 room-9-square-26)
 		(trans room-9-square-36 room-9-square-48)
 
  		 ;;; transitions for room-9-square-37
 		(trans room-9-square-37 room-9-square-36)
 		(trans room-9-square-37 room-9-square-38)
 		(trans room-9-square-37 room-9-square-26)
 		(trans room-9-square-37 room-9-square-48)
 
  		 ;;; transitions for room-9-square-38
 		(trans room-9-square-38 room-9-square-37)
 		(trans room-9-square-38 room-9-square-39)
 		(trans room-9-square-38 room-9-square-27)
 		(trans room-9-square-38 room-9-square-49)
 
  		 ;;; transitions for room-9-square-39
 		(trans room-9-square-39 room-9-square-38)
 		(trans room-9-square-39 room-9-square-40)
 		(trans room-9-square-39 room-9-square-28)
 		(trans room-9-square-39 room-9-square-50)
 
  		 ;;; transitions for room-9-square-40
 		(trans room-9-square-40 room-9-square-39)
 		(trans room-9-square-40 room-9-square-41)
 		(trans room-9-square-40 room-9-square-29)
 		(trans room-9-square-40 room-9-square-51)
 
  		 ;;; transitions for room-9-square-41
 		(trans room-9-square-41 room-9-square-40)
 		(trans room-9-square-41 room-9-square-42)
 		(trans room-9-square-41 room-9-square-30)
 		(trans room-9-square-41 room-9-square-52)
 
  		 ;;; transitions for room-9-square-42
 		(trans room-9-square-42 room-9-square-41)
 		(trans room-9-square-42 room-9-square-43)
 		(trans room-9-square-42 room-9-square-31)
 		(trans room-9-square-42 room-9-square-53)
 
  		 ;;; transitions for room-9-square-43
 		(trans room-9-square-43 room-9-square-42)
 		(trans room-9-square-43 room-9-square-44)
 		(trans room-9-square-43 room-9-square-32)
 		(trans room-9-square-43 room-9-square-54)
 
  		 ;;; transitions for room-9-square-44
 		(trans room-9-square-44 room-9-square-43)
 		(trans room-9-square-44 room-9-square-45)
 		(trans room-9-square-44 room-9-square-33)
 		(trans room-9-square-44 room-9-square-55)
 
  		 ;;; transitions for room-9-square-45
 		(trans room-9-square-45 room-9-square-44)
 		(trans room-9-square-45 room-9-square-46)
 		(trans room-9-square-45 room-9-square-34)
 		(trans room-9-square-45 room-9-square-56)
 
  		 ;;; transitions for room-9-square-46
 		(trans room-9-square-46 room-9-square-45)
 		(trans room-9-square-46 room-9-square-47)
 		(trans room-9-square-46 room-9-square-35)
 		(trans room-9-square-46 room-9-square-57)
 
  		 ;;; transitions for room-9-square-47
 		(trans room-9-square-47 room-9-square-46)
 		(trans room-9-square-47 room-9-square-35)
 		(trans room-9-square-47 room-9-square-57)
 
  		 ;;; transitions for room-2-square-2
 		(trans room-2-square-2 room-1-square-20)
 		(trans room-2-square-2 room-2-square-1)
 		(trans room-2-square-2 room-2-square-3)
 		(trans room-2-square-2 room-2-square-4)
 		(trans room-2-square-2 room-2-square-10)
 
  		 ;;; transitions for room-2-square-1
 		(trans room-2-square-1 room-1-square-20)
 		(trans room-2-square-1 room-2-square-2)
 		(trans room-2-square-1 room-2-square-3)
 
  		 ;;; transitions for room-2-square-3
 		(trans room-2-square-3 room-1-square-20)
 		(trans room-2-square-3 room-2-square-2)
 		(trans room-2-square-3 room-2-square-1)
 		(trans room-2-square-3 room-2-square-8)
 		(trans room-2-square-3 room-2-square-9)
 
  		 ;;; transitions for room-2-square-4
 		(trans room-2-square-4 room-6-square-12)
 		(trans room-2-square-4 room-2-square-2)
 		(trans room-2-square-4 room-2-square-11)
 
  		 ;;; transitions for room-6-square-12
 		(trans room-6-square-12 room-2-square-4)
 		(trans room-6-square-12 room-6-square-6)
 		(trans room-6-square-12 room-6-square-13)
 
  		 ;;; transitions for room-2-square-8
 		(trans room-2-square-8 room-2-square-3)
 		(trans room-2-square-8 room-2-square-7)
 		(trans room-2-square-8 room-2-square-9)
 		(trans room-2-square-8 room-2-square-13)
 
  		 ;;; transitions for room-2-square-7
 		(trans room-2-square-7 room-2-square-8)
 		(trans room-2-square-7 room-2-square-6)
 		(trans room-2-square-7 room-2-square-13)
 
  		 ;;; transitions for room-2-square-6
 		(trans room-2-square-6 room-2-square-7)
 		(trans room-2-square-6 room-2-square-5)
 		(trans room-2-square-6 room-2-square-12)
 
  		 ;;; transitions for room-2-square-5
 		(trans room-2-square-5 room-2-square-6)
 		(trans room-2-square-5 room-2-square-12)
 
  		 ;;; transitions for room-2-square-9
 		(trans room-2-square-9 room-2-square-8)
 		(trans room-2-square-9 room-2-square-10)
 		(trans room-2-square-9 room-2-square-3)
 
  		 ;;; transitions for room-2-square-10
 		(trans room-2-square-10 room-2-square-9)
 		(trans room-2-square-10 room-2-square-11)
 		(trans room-2-square-10 room-2-square-2)
 
  		 ;;; transitions for room-2-square-11
 		(trans room-2-square-11 room-2-square-10)
 		(trans room-2-square-11 room-6-square-13)
 		(trans room-2-square-11 room-2-square-4)
 
  		 ;;; transitions for room-6-square-13
 		(trans room-6-square-13 room-2-square-11)
 		(trans room-6-square-13 room-6-square-14)
 		(trans room-6-square-13 room-6-square-12)
 		(trans room-6-square-13 room-6-square-19)
 
  		 ;;; transitions for room-6-square-14
 		(trans room-6-square-14 room-6-square-13)
 		(trans room-6-square-14 room-6-square-15)
 		(trans room-6-square-14 room-6-square-6)
 		(trans room-6-square-14 room-6-square-19)
 
  		 ;;; transitions for room-6-square-15
 		(trans room-6-square-15 room-6-square-14)
 		(trans room-6-square-15 room-6-square-16)
 		(trans room-6-square-15 room-6-square-7)
 		(trans room-6-square-15 room-6-square-19)
 		(trans room-6-square-15 room-4-square-1)
 		(trans room-6-square-15 room-7-square-1)
 
  		 ;;; transitions for room-6-square-16
 		(trans room-6-square-16 room-6-square-15)
 		(trans room-6-square-16 room-6-square-17)
 		(trans room-6-square-16 room-6-square-8)
 		(trans room-6-square-16 room-7-square-2)
 
  		 ;;; transitions for room-6-square-17
 		(trans room-6-square-17 room-6-square-16)
 		(trans room-6-square-17 room-6-square-18)
 		(trans room-6-square-17 room-6-square-9)
 		(trans room-6-square-17 room-7-square-3)
 
  		 ;;; transitions for room-6-square-18
 		(trans room-6-square-18 room-6-square-17)
 		(trans room-6-square-18 room-6-square-11)
 		(trans room-6-square-18 room-6-square-10)
 
  		 ;;; transitions for room-9-square-48
 		(trans room-9-square-48 room-9-square-36)
 		(trans room-9-square-48 room-9-square-49)
 		(trans room-9-square-48 room-9-square-37)
 		(trans room-9-square-48 room-9-square-58)
 
  		 ;;; transitions for room-9-square-49
 		(trans room-9-square-49 room-9-square-48)
 		(trans room-9-square-49 room-9-square-50)
 		(trans room-9-square-49 room-9-square-38)
 		(trans room-9-square-49 room-9-square-58)
 
  		 ;;; transitions for room-9-square-50
 		(trans room-9-square-50 room-9-square-49)
 		(trans room-9-square-50 room-9-square-51)
 		(trans room-9-square-50 room-9-square-39)
 		(trans room-9-square-50 room-9-square-59)
 
  		 ;;; transitions for room-9-square-51
 		(trans room-9-square-51 room-9-square-50)
 		(trans room-9-square-51 room-9-square-52)
 		(trans room-9-square-51 room-9-square-40)
 		(trans room-9-square-51 room-9-square-60)
 
  		 ;;; transitions for room-9-square-52
 		(trans room-9-square-52 room-9-square-51)
 		(trans room-9-square-52 room-9-square-53)
 		(trans room-9-square-52 room-9-square-41)
 		(trans room-9-square-52 room-9-square-61)
 
  		 ;;; transitions for room-9-square-53
 		(trans room-9-square-53 room-9-square-52)
 		(trans room-9-square-53 room-9-square-54)
 		(trans room-9-square-53 room-9-square-42)
 		(trans room-9-square-53 room-9-square-62)
 
  		 ;;; transitions for room-9-square-54
 		(trans room-9-square-54 room-9-square-53)
 		(trans room-9-square-54 room-9-square-55)
 		(trans room-9-square-54 room-9-square-43)
 		(trans room-9-square-54 room-9-square-63)
 
  		 ;;; transitions for room-9-square-55
 		(trans room-9-square-55 room-9-square-54)
 		(trans room-9-square-55 room-9-square-56)
 		(trans room-9-square-55 room-9-square-44)
 		(trans room-9-square-55 room-9-square-64)
 
  		 ;;; transitions for room-9-square-56
 		(trans room-9-square-56 room-9-square-55)
 		(trans room-9-square-56 room-9-square-57)
 		(trans room-9-square-56 room-9-square-45)
 		(trans room-9-square-56 room-9-square-65)
 
  		 ;;; transitions for room-9-square-57
 		(trans room-9-square-57 room-9-square-56)
 		(trans room-9-square-57 room-9-square-47)
 		(trans room-9-square-57 room-9-square-65)
 		(trans room-9-square-57 room-9-square-46)
 
  		 ;;; transitions for room-7-square-3
 		(trans room-7-square-3 room-6-square-17)
 		(trans room-7-square-3 room-7-square-2)
 		(trans room-7-square-3 room-7-square-5)
 
  		 ;;; transitions for room-7-square-2
 		(trans room-7-square-2 room-6-square-16)
 		(trans room-7-square-2 room-7-square-3)
 		(trans room-7-square-2 room-7-square-1)
 		(trans room-7-square-2 room-7-square-4)
 
  		 ;;; transitions for room-7-square-1
 		(trans room-7-square-1 room-7-square-2)
 		(trans room-7-square-1 room-6-square-15)
 		(trans room-7-square-1 room-7-square-4)
 
  		 ;;; transitions for room-2-square-12
 		(trans room-2-square-12 room-2-square-13)
 		(trans room-2-square-12 room-2-square-6)
 		(trans room-2-square-12 room-3-square-1)
 		(trans room-2-square-12 room-3-square-4)
 		(trans room-2-square-12 room-2-square-5)
 
  		 ;;; transitions for room-2-square-13
 		(trans room-2-square-13 room-2-square-12)
 		(trans room-2-square-13 room-2-square-8)
 		(trans room-2-square-13 room-2-square-7)
 		(trans room-2-square-13 room-3-square-2)
 
  		 ;;; transitions for room-6-square-19
 		(trans room-6-square-19 room-6-square-13)
 		(trans room-6-square-19 room-6-square-15)
 		(trans room-6-square-19 room-6-square-14)
 		(trans room-6-square-19 room-4-square-1)
 
  		 ;;; transitions for room-7-square-4
 		(trans room-7-square-4 room-7-square-5)
 		(trans room-7-square-4 room-7-square-2)
 		(trans room-7-square-4 room-7-square-6)
 		(trans room-7-square-4 room-7-square-1)
 
  		 ;;; transitions for room-7-square-5
 		(trans room-7-square-5 room-7-square-4)
 		(trans room-7-square-5 room-7-square-3)
 		(trans room-7-square-5 room-7-square-7)
 
  		 ;;; transitions for room-9-square-58
 		(trans room-9-square-58 room-9-square-48)
 		(trans room-9-square-58 room-9-square-59)
 		(trans room-9-square-58 room-9-square-49)
 		(trans room-9-square-58 room-9-square-66)
 
  		 ;;; transitions for room-9-square-59
 		(trans room-9-square-59 room-9-square-58)
 		(trans room-9-square-59 room-9-square-60)
 		(trans room-9-square-59 room-9-square-50)
 		(trans room-9-square-59 room-9-square-66)
 
  		 ;;; transitions for room-9-square-60
 		(trans room-9-square-60 room-9-square-59)
 		(trans room-9-square-60 room-9-square-61)
 		(trans room-9-square-60 room-9-square-51)
 		(trans room-9-square-60 room-9-square-67)
 
  		 ;;; transitions for room-9-square-61
 		(trans room-9-square-61 room-9-square-60)
 		(trans room-9-square-61 room-9-square-62)
 		(trans room-9-square-61 room-9-square-52)
 		(trans room-9-square-61 room-9-square-68)
 
  		 ;;; transitions for room-9-square-62
 		(trans room-9-square-62 room-9-square-61)
 		(trans room-9-square-62 room-9-square-63)
 		(trans room-9-square-62 room-9-square-53)
 		(trans room-9-square-62 room-9-square-69)
 
  		 ;;; transitions for room-9-square-63
 		(trans room-9-square-63 room-9-square-62)
 		(trans room-9-square-63 room-9-square-64)
 		(trans room-9-square-63 room-9-square-54)
 		(trans room-9-square-63 room-9-square-70)
 
  		 ;;; transitions for room-9-square-64
 		(trans room-9-square-64 room-9-square-63)
 		(trans room-9-square-64 room-9-square-65)
 		(trans room-9-square-64 room-9-square-55)
 		(trans room-9-square-64 room-9-square-71)
 
  		 ;;; transitions for room-9-square-65
 		(trans room-9-square-65 room-9-square-64)
 		(trans room-9-square-65 room-9-square-57)
 		(trans room-9-square-65 room-9-square-71)
 		(trans room-9-square-65 room-9-square-56)
 
  		 ;;; transitions for room-4-square-1
 		(trans room-4-square-1 room-6-square-19)
 		(trans room-4-square-1 room-6-square-15)
 		(trans room-4-square-1 room-4-square-2)
 		(trans room-4-square-1 room-4-square-10)
 		(trans room-4-square-1 room-4-square-9)
 
  		 ;;; transitions for room-4-square-2
 		(trans room-4-square-2 room-4-square-1)
 		(trans room-4-square-2 room-4-square-3)
 		(trans room-4-square-2 room-4-square-8)
 
  		 ;;; transitions for room-3-square-2
 		(trans room-3-square-2 room-2-square-13)
 		(trans room-3-square-2 room-3-square-1)
 		(trans room-3-square-2 room-3-square-4)
 		(trans room-3-square-2 room-3-square-5)
 
  		 ;;; transitions for room-4-square-3
 		(trans room-4-square-3 room-4-square-2)
 		(trans room-4-square-3 room-4-square-4)
 		(trans room-4-square-3 room-4-square-7)
 
  		 ;;; transitions for room-4-square-4
 		(trans room-4-square-4 room-4-square-3)
 		(trans room-4-square-4 room-4-square-5)
 		(trans room-4-square-4 room-4-square-6)
 
  		 ;;; transitions for room-4-square-5
 		(trans room-4-square-5 room-4-square-4)
 		(trans room-4-square-5 room-4-square-6)
 		(trans room-4-square-5 room-4-square-11)
 
  		 ;;; transitions for room-3-square-1
 		(trans room-3-square-1 room-2-square-12)
 		(trans room-3-square-1 room-3-square-2)
 		(trans room-3-square-1 room-3-square-4)
 
  		 ;;; transitions for room-3-square-4
 		(trans room-3-square-4 room-3-square-2)
 		(trans room-3-square-4 room-3-square-1)
 		(trans room-3-square-4 room-3-square-3)
 		(trans room-3-square-4 room-3-square-5)
 		(trans room-3-square-4 room-3-square-6)
 		(trans room-3-square-4 room-2-square-12)
 
  		 ;;; transitions for room-3-square-3
 		(trans room-3-square-3 room-3-square-4)
 		(trans room-3-square-3 room-3-square-6)
 
  		 ;;; transitions for room-3-square-5
 		(trans room-3-square-5 room-3-square-4)
 		(trans room-3-square-5 room-3-square-7)
 		(trans room-3-square-5 room-3-square-2)
 
  		 ;;; transitions for room-4-square-6
 		(trans room-4-square-6 room-4-square-5)
 		(trans room-4-square-6 room-4-square-7)
 		(trans room-4-square-6 room-4-square-4)
 		(trans room-4-square-6 room-4-square-12)
 
  		 ;;; transitions for room-4-square-7
 		(trans room-4-square-7 room-4-square-6)
 		(trans room-4-square-7 room-4-square-8)
 		(trans room-4-square-7 room-4-square-3)
 		(trans room-4-square-7 room-4-square-13)
 
  		 ;;; transitions for room-4-square-8
 		(trans room-4-square-8 room-4-square-7)
 		(trans room-4-square-8 room-4-square-9)
 		(trans room-4-square-8 room-4-square-2)
 		(trans room-4-square-8 room-4-square-14)
 
  		 ;;; transitions for room-4-square-9
 		(trans room-4-square-9 room-4-square-8)
 		(trans room-4-square-9 room-4-square-10)
 		(trans room-4-square-9 room-4-square-1)
 		(trans room-4-square-9 room-4-square-15)
 
  		 ;;; transitions for room-4-square-10
 		(trans room-4-square-10 room-4-square-9)
 		(trans room-4-square-10 room-4-square-1)
 		(trans room-4-square-10 room-4-square-16)
 
  		 ;;; transitions for room-7-square-6
 		(trans room-7-square-6 room-7-square-7)
 		(trans room-7-square-6 room-7-square-4)
 		(trans room-7-square-6 room-8-square-1)
 		(trans room-7-square-6 room-8-square-2)
 
  		 ;;; transitions for room-7-square-7
 		(trans room-7-square-7 room-7-square-6)
 		(trans room-7-square-7 room-7-square-5)
 		(trans room-7-square-7 room-8-square-3)
 
  		 ;;; transitions for room-9-square-66
 		(trans room-9-square-66 room-9-square-58)
 		(trans room-9-square-66 room-9-square-67)
 		(trans room-9-square-66 room-9-square-59)
 
  		 ;;; transitions for room-9-square-67
 		(trans room-9-square-67 room-9-square-66)
 		(trans room-9-square-67 room-9-square-68)
 		(trans room-9-square-67 room-9-square-60)
 		(trans room-9-square-67 room-9-square-72)
 
  		 ;;; transitions for room-9-square-68
 		(trans room-9-square-68 room-9-square-67)
 		(trans room-9-square-68 room-9-square-69)
 		(trans room-9-square-68 room-9-square-61)
 		(trans room-9-square-68 room-9-square-72)
 
  		 ;;; transitions for room-9-square-69
 		(trans room-9-square-69 room-9-square-68)
 		(trans room-9-square-69 room-9-square-70)
 		(trans room-9-square-69 room-9-square-62)
 		(trans room-9-square-69 room-9-square-73)
 
  		 ;;; transitions for room-9-square-70
 		(trans room-9-square-70 room-9-square-69)
 		(trans room-9-square-70 room-9-square-71)
 		(trans room-9-square-70 room-9-square-63)
 		(trans room-9-square-70 room-9-square-74)
 
  		 ;;; transitions for room-9-square-71
 		(trans room-9-square-71 room-9-square-70)
 		(trans room-9-square-71 room-9-square-65)
 		(trans room-9-square-71 room-13-square-2)
 		(trans room-9-square-71 room-13-square-1)
 		(trans room-9-square-71 room-9-square-74)
 		(trans room-9-square-71 room-9-square-64)
 
  		 ;;; transitions for room-8-square-3
 		(trans room-8-square-3 room-7-square-7)
 		(trans room-8-square-3 room-8-square-1)
 		(trans room-8-square-3 room-8-square-2)
 		(trans room-8-square-3 room-8-square-4)
 		(trans room-8-square-3 room-8-square-6)
 
  		 ;;; transitions for room-13-square-1
 		(trans room-13-square-1 room-9-square-71)
 		(trans room-13-square-1 room-13-square-2)
 
  		 ;;; transitions for room-13-square-2
 		(trans room-13-square-2 room-13-square-1)
 		(trans room-13-square-2 room-13-square-3)
 		(trans room-13-square-2 room-9-square-71)
 
  		 ;;; transitions for room-8-square-1
 		(trans room-8-square-1 room-7-square-6)
 		(trans room-8-square-1 room-8-square-3)
 		(trans room-8-square-1 room-8-square-2)
 
  		 ;;; transitions for room-8-square-2
 		(trans room-8-square-2 room-7-square-6)
 		(trans room-8-square-2 room-8-square-3)
 		(trans room-8-square-2 room-8-square-1)
 		(trans room-8-square-2 room-8-square-5)
 
  		 ;;; transitions for room-3-square-6
 		(trans room-3-square-6 room-3-square-7)
 		(trans room-3-square-6 room-3-square-4)
 		(trans room-3-square-6 room-3-square-8)
 		(trans room-3-square-6 room-3-square-3)
 
  		 ;;; transitions for room-3-square-7
 		(trans room-3-square-7 room-3-square-6)
 		(trans room-3-square-7 room-3-square-5)
 		(trans room-3-square-7 room-3-square-9)
 
  		 ;;; transitions for room-4-square-11
 		(trans room-4-square-11 room-4-square-12)
 		(trans room-4-square-11 room-4-square-5)
 		(trans room-4-square-11 room-4-square-17)
 
  		 ;;; transitions for room-4-square-12
 		(trans room-4-square-12 room-4-square-11)
 		(trans room-4-square-12 room-4-square-13)
 		(trans room-4-square-12 room-4-square-6)
 		(trans room-4-square-12 room-4-square-18)
 
  		 ;;; transitions for room-4-square-13
 		(trans room-4-square-13 room-4-square-12)
 		(trans room-4-square-13 room-4-square-14)
 		(trans room-4-square-13 room-4-square-7)
 		(trans room-4-square-13 room-4-square-19)
 
  		 ;;; transitions for room-4-square-14
 		(trans room-4-square-14 room-4-square-13)
 		(trans room-4-square-14 room-4-square-15)
 		(trans room-4-square-14 room-4-square-8)
 		(trans room-4-square-14 room-4-square-20)
 
  		 ;;; transitions for room-4-square-15
 		(trans room-4-square-15 room-4-square-14)
 		(trans room-4-square-15 room-4-square-16)
 		(trans room-4-square-15 room-4-square-9)
 		(trans room-4-square-15 room-4-square-21)
 
  		 ;;; transitions for room-4-square-16
 		(trans room-4-square-16 room-4-square-15)
 		(trans room-4-square-16 room-4-square-10)
 		(trans room-4-square-16 room-4-square-22)
 
  		 ;;; transitions for room-8-square-4
 		(trans room-8-square-4 room-8-square-3)
 		(trans room-8-square-4 room-8-square-8)
 		(trans room-8-square-4 room-8-square-7)
 
  		 ;;; transitions for room-9-square-72
 		(trans room-9-square-72 room-9-square-67)
 		(trans room-9-square-72 room-9-square-73)
 		(trans room-9-square-72 room-9-square-68)
 		(trans room-9-square-72 room-9-square-75)
 
  		 ;;; transitions for room-9-square-73
 		(trans room-9-square-73 room-9-square-72)
 		(trans room-9-square-73 room-9-square-74)
 		(trans room-9-square-73 room-9-square-69)
 		(trans room-9-square-73 room-9-square-75)
 
  		 ;;; transitions for room-9-square-74
 		(trans room-9-square-74 room-9-square-73)
 		(trans room-9-square-74 room-9-square-71)
 		(trans room-9-square-74 room-9-square-76)
 		(trans room-9-square-74 room-9-square-70)
 
  		 ;;; transitions for room-13-square-3
 		(trans room-13-square-3 room-13-square-2)
 
  		 ;;; transitions for room-8-square-8
 		(trans room-8-square-8 room-8-square-4)
 		(trans room-8-square-8 room-8-square-7)
 		(trans room-8-square-8 room-8-square-13)
 		(trans room-8-square-8 room-8-square-12)
 
  		 ;;; transitions for room-3-square-8
 		(trans room-3-square-8 room-3-square-9)
 		(trans room-3-square-8 room-3-square-6)
 		(trans room-3-square-8 room-3-square-10)
 
  		 ;;; transitions for room-3-square-9
 		(trans room-3-square-9 room-3-square-8)
 		(trans room-3-square-9 room-3-square-7)
 		(trans room-3-square-9 room-3-square-11)
 
  		 ;;; transitions for room-4-square-17
 		(trans room-4-square-17 room-4-square-18)
 		(trans room-4-square-17 room-4-square-11)
 		(trans room-4-square-17 room-4-square-23)
 
  		 ;;; transitions for room-4-square-18
 		(trans room-4-square-18 room-4-square-17)
 		(trans room-4-square-18 room-4-square-19)
 		(trans room-4-square-18 room-4-square-12)
 		(trans room-4-square-18 room-4-square-23)
 
  		 ;;; transitions for room-4-square-19
 		(trans room-4-square-19 room-4-square-18)
 		(trans room-4-square-19 room-4-square-20)
 		(trans room-4-square-19 room-4-square-13)
 		(trans room-4-square-19 room-4-square-24)
 
  		 ;;; transitions for room-4-square-20
 		(trans room-4-square-20 room-4-square-19)
 		(trans room-4-square-20 room-4-square-21)
 		(trans room-4-square-20 room-4-square-14)
 		(trans room-4-square-20 room-4-square-25)
 
  		 ;;; transitions for room-4-square-21
 		(trans room-4-square-21 room-4-square-20)
 		(trans room-4-square-21 room-4-square-22)
 		(trans room-4-square-21 room-4-square-15)
 		(trans room-4-square-21 room-4-square-26)
 
  		 ;;; transitions for room-4-square-22
 		(trans room-4-square-22 room-4-square-21)
 		(trans room-4-square-22 room-4-square-16)
 		(trans room-4-square-22 room-4-square-27)
 
  		 ;;; transitions for room-8-square-5
 		(trans room-8-square-5 room-8-square-6)
 		(trans room-8-square-5 room-8-square-2)
 		(trans room-8-square-5 room-8-square-9)
 
  		 ;;; transitions for room-8-square-6
 		(trans room-8-square-6 room-8-square-5)
 		(trans room-8-square-6 room-8-square-7)
 		(trans room-8-square-6 room-8-square-3)
 		(trans room-8-square-6 room-8-square-10)
 
  		 ;;; transitions for room-8-square-7
 		(trans room-8-square-7 room-8-square-6)
 		(trans room-8-square-7 room-8-square-8)
 		(trans room-8-square-7 room-8-square-4)
 		(trans room-8-square-7 room-8-square-11)
 
  		 ;;; transitions for room-9-square-75
 		(trans room-9-square-75 room-9-square-72)
 		(trans room-9-square-75 room-9-square-76)
 		(trans room-9-square-75 room-9-square-73)
 		(trans room-9-square-75 room-9-square-77)
 
  		 ;;; transitions for room-9-square-76
 		(trans room-9-square-76 room-9-square-75)
 		(trans room-9-square-76 room-9-square-74)
 		(trans room-9-square-76 room-9-square-78)
 
  		 ;;; transitions for room-8-square-13
 		(trans room-8-square-13 room-8-square-8)
 		(trans room-8-square-13 room-8-square-12)
 		(trans room-8-square-13 room-8-square-18)
 		(trans room-8-square-13 room-8-square-17)
 
  		 ;;; transitions for room-3-square-10
 		(trans room-3-square-10 room-3-square-11)
 		(trans room-3-square-10 room-3-square-8)
 		(trans room-3-square-10 room-2-square-14)
 
  		 ;;; transitions for room-3-square-11
 		(trans room-3-square-11 room-3-square-10)
 		(trans room-3-square-11 room-3-square-9)
 
  		 ;;; transitions for room-4-square-23
 		(trans room-4-square-23 room-4-square-24)
 		(trans room-4-square-23 room-4-square-18)
 		(trans room-4-square-23 room-4-square-17)
 
  		 ;;; transitions for room-4-square-24
 		(trans room-4-square-24 room-4-square-23)
 		(trans room-4-square-24 room-4-square-25)
 		(trans room-4-square-24 room-4-square-19)
 
  		 ;;; transitions for room-4-square-25
 		(trans room-4-square-25 room-4-square-24)
 		(trans room-4-square-25 room-4-square-26)
 		(trans room-4-square-25 room-4-square-20)
 
  		 ;;; transitions for room-4-square-26
 		(trans room-4-square-26 room-4-square-25)
 		(trans room-4-square-26 room-4-square-27)
 		(trans room-4-square-26 room-4-square-21)
 
  		 ;;; transitions for room-4-square-27
 		(trans room-4-square-27 room-4-square-26)
 		(trans room-4-square-27 room-4-square-22)
 
  		 ;;; transitions for room-8-square-9
 		(trans room-8-square-9 room-8-square-10)
 		(trans room-8-square-9 room-8-square-5)
 		(trans room-8-square-9 room-8-square-14)
 
  		 ;;; transitions for room-8-square-10
 		(trans room-8-square-10 room-8-square-9)
 		(trans room-8-square-10 room-8-square-11)
 		(trans room-8-square-10 room-8-square-6)
 		(trans room-8-square-10 room-8-square-14)
 
  		 ;;; transitions for room-8-square-11
 		(trans room-8-square-11 room-8-square-10)
 		(trans room-8-square-11 room-8-square-12)
 		(trans room-8-square-11 room-8-square-7)
 		(trans room-8-square-11 room-8-square-15)
 
  		 ;;; transitions for room-8-square-12
 		(trans room-8-square-12 room-8-square-11)
 		(trans room-8-square-12 room-8-square-13)
 		(trans room-8-square-12 room-8-square-8)
 		(trans room-8-square-12 room-8-square-16)
 
  		 ;;; transitions for room-9-square-77
 		(trans room-9-square-77 room-9-square-75)
 		(trans room-9-square-77 room-9-square-78)
 		(trans room-9-square-77 room-9-square-79)
 		(trans room-9-square-77 room-12-square-2)
 
  		 ;;; transitions for room-9-square-78
 		(trans room-9-square-78 room-9-square-77)
 		(trans room-9-square-78 room-9-square-76)
 		(trans room-9-square-78 room-12-square-1)
 		(trans room-9-square-78 room-12-square-3)
 
  		 ;;; transitions for room-8-square-18
 		(trans room-8-square-18 room-8-square-13)
 		(trans room-8-square-18 room-8-square-17)
 		(trans room-8-square-18 room-8-square-19)
 		(trans room-8-square-18 room-8-square-23)
 
  		 ;;; transitions for room-12-square-1
 		(trans room-12-square-1 room-9-square-78)
 		(trans room-12-square-1 room-12-square-3)
 
  		 ;;; transitions for room-2-square-14
 		(trans room-2-square-14 room-3-square-10)
 
  		 ;;; transitions for room-8-square-14
 		(trans room-8-square-14 room-8-square-9)
 		(trans room-8-square-14 room-8-square-15)
 		(trans room-8-square-14 room-8-square-10)
 		(trans room-8-square-14 room-8-square-20)
 
  		 ;;; transitions for room-8-square-15
 		(trans room-8-square-15 room-8-square-14)
 		(trans room-8-square-15 room-8-square-16)
 		(trans room-8-square-15 room-8-square-11)
 		(trans room-8-square-15 room-8-square-20)
 
  		 ;;; transitions for room-8-square-16
 		(trans room-8-square-16 room-8-square-15)
 		(trans room-8-square-16 room-8-square-17)
 		(trans room-8-square-16 room-8-square-12)
 		(trans room-8-square-16 room-8-square-21)
 
  		 ;;; transitions for room-8-square-17
 		(trans room-8-square-17 room-8-square-16)
 		(trans room-8-square-17 room-8-square-18)
 		(trans room-8-square-17 room-8-square-13)
 		(trans room-8-square-17 room-8-square-22)
 
  		 ;;; transitions for room-9-square-79
 		(trans room-9-square-79 room-12-square-2)
 		(trans room-9-square-79 room-9-square-77)
 
  		 ;;; transitions for room-12-square-2
 		(trans room-12-square-2 room-9-square-79)
 		(trans room-12-square-2 room-12-square-3)
 		(trans room-12-square-2 room-9-square-77)
 		(trans room-12-square-2 room-12-square-7)
 		(trans room-12-square-2 room-12-square-8)
 
  		 ;;; transitions for room-12-square-3
 		(trans room-12-square-3 room-12-square-2)
 		(trans room-12-square-3 room-12-square-10)
 		(trans room-12-square-3 room-12-square-9)
 		(trans room-12-square-3 room-12-square-1)
 		(trans room-12-square-3 room-9-square-78)
 
  		 ;;; transitions for room-8-square-19
 		(trans room-8-square-19 room-8-square-18)
 		(trans room-8-square-19 room-12-square-4)
 		(trans room-8-square-19 room-8-square-25)
 		(trans room-8-square-19 room-8-square-24)
 
  		 ;;; transitions for room-12-square-10
 		(trans room-12-square-10 room-12-square-3)
 		(trans room-12-square-10 room-12-square-9)
 		(trans room-12-square-10 room-12-square-14)
 
  		 ;;; transitions for room-12-square-7
 		(trans room-12-square-7 room-12-square-6)
 		(trans room-12-square-7 room-12-square-2)
 		(trans room-12-square-7 room-12-square-8)
 	;;;(trans room-12-square-7 room-12-square-12)
 
  		 ;;; transitions for room-12-square-6
 		(trans room-12-square-6 room-12-square-7)
 		(trans room-12-square-6 room-8-square-25)
 		(trans room-12-square-6 room-12-square-4)
 		(trans room-12-square-6 room-8-square-24)
 		(trans room-12-square-6 room-12-square-5)
 		(trans room-12-square-6 room-12-square-11)
 
  		 ;;; transitions for room-12-square-4
 		(trans room-12-square-4 room-8-square-19)
 		(trans room-12-square-4 room-12-square-6)
 
  		 ;;; transitions for room-8-square-20
 		(trans room-8-square-20 room-8-square-14)
 		(trans room-8-square-20 room-8-square-21)
 		(trans room-8-square-20 room-8-square-15)
 
  		 ;;; transitions for room-8-square-21
 		(trans room-8-square-21 room-8-square-20)
 		(trans room-8-square-21 room-8-square-22)
 		(trans room-8-square-21 room-8-square-16)
 		(trans room-8-square-21 room-8-square-26)
 
  		 ;;; transitions for room-8-square-22
 		(trans room-8-square-22 room-8-square-21)
 		(trans room-8-square-22 room-8-square-23)
 		(trans room-8-square-22 room-8-square-17)
 		(trans room-8-square-22 room-8-square-26)
 
  		 ;;; transitions for room-8-square-23
 		(trans room-8-square-23 room-8-square-22)
 		(trans room-8-square-23 room-8-square-24)
 		(trans room-8-square-23 room-8-square-18)
 		(trans room-8-square-23 room-8-square-26)
 		(trans room-8-square-23 room-11-square-3)
 
  		 ;;; transitions for room-8-square-24
 		(trans room-8-square-24 room-8-square-23)
 		(trans room-8-square-24 room-8-square-25)
 		(trans room-8-square-24 room-8-square-19)
 		(trans room-8-square-24 room-12-square-6)
 		(trans room-8-square-24 room-12-square-5)
 		(trans room-8-square-24 room-11-square-1)
 
  		 ;;; transitions for room-8-square-25
 		(trans room-8-square-25 room-8-square-24)
 		(trans room-8-square-25 room-12-square-6)
 		(trans room-8-square-25 room-8-square-19)
 
  		 ;;; transitions for room-12-square-8
 		(trans room-12-square-8 room-12-square-7)
 		(trans room-12-square-8 room-12-square-9)
 		(trans room-12-square-8 room-12-square-2)
 		(trans room-12-square-8 room-12-square-13)
 
  		 ;;; transitions for room-12-square-9
 		(trans room-12-square-9 room-12-square-8)
 		(trans room-12-square-9 room-12-square-10)
 		(trans room-12-square-9 room-12-square-3)
 		(trans room-12-square-9 room-12-square-14)
 
  		 ;;; transitions for room-12-square-5
 		(trans room-12-square-5 room-8-square-24)
 		(trans room-12-square-5 room-12-square-6)
 
  		 ;;; transitions for room-11-square-3
 		(trans room-11-square-3 room-8-square-23)
 		(trans room-11-square-3 room-11-square-4)
 		(trans room-11-square-3 room-11-square-1)
 
  		 ;;; transitions for room-11-square-1
 		(trans room-11-square-1 room-11-square-3)
 		(trans room-11-square-1 room-8-square-24)
 		(trans room-11-square-1 room-11-square-2)
 		(trans room-11-square-1 room-11-square-4)
 
  		 ;;; transitions for room-11-square-2
 		(trans room-11-square-2 room-11-square-1)
 		(trans room-11-square-2 room-8-square-25)
 		(trans room-11-square-2 room-11-square-4)
 
  		 ;;; transitions for room-8-square-26
 		(trans room-8-square-26 room-8-square-21)
 		(trans room-8-square-26 room-8-square-23)
 		(trans room-8-square-26 room-8-square-22)
 
  		 ;;; transitions for room-11-square-4
 		(trans room-11-square-4 room-11-square-3)
 		(trans room-11-square-4 room-11-square-2)
 		(trans room-11-square-4 room-11-square-1)
 
  		 ;;; transitions for room-12-square-11
 		(trans room-12-square-11 room-12-square-6)
 		(trans room-12-square-11 room-12-square-12)
 
  		 ;;; transitions for room-12-square-12
 		(trans room-12-square-12 room-12-square-11)
 		(trans room-12-square-12 room-12-square-13)
 		(trans room-12-square-12 room-12-square-7)
 
  		 ;;; transitions for room-12-square-13
 		(trans room-12-square-13 room-12-square-12)
 		(trans room-12-square-13 room-12-square-14)
 		(trans room-12-square-13 room-12-square-8)
 		(trans room-12-square-13 room-10-square-2)
 	;;;(trans room-12-square-13 room-10-square-3)
 
  		 ;;; transitions for room-12-square-14
 		(trans room-12-square-14 room-12-square-13)
 		(trans room-12-square-14 room-12-square-10)
 	;;;(trans room-12-square-14 room-10-square-3)
 		(trans room-12-square-14 room-12-square-9)
 
  		 ;;; transitions for room-10-square-8
 		(trans room-10-square-8 room-10-square-7)
 		(trans room-10-square-8 room-10-square-20)
 
  		 ;;; transitions for room-10-square-7
 		(trans room-10-square-7 room-10-square-8)
 		(trans room-10-square-7 room-10-square-6)
 		(trans room-10-square-7 room-10-square-19)
 
  		 ;;; transitions for room-10-square-6
 		(trans room-10-square-6 room-10-square-7)
 		(trans room-10-square-6 room-10-square-5)
 		(trans room-10-square-6 room-10-square-18)
 
  		 ;;; transitions for room-10-square-5
 		(trans room-10-square-5 room-10-square-6)
 		(trans room-10-square-5 room-10-square-4)
 		(trans room-10-square-5 room-10-square-17)
 
  		 ;;; transitions for room-10-square-4
 		(trans room-10-square-4 room-10-square-5)
 		(trans room-10-square-4 room-10-square-3)
 		(trans room-10-square-4 room-10-square-16)
 
  		 ;;; transitions for room-10-square-3
 		(trans room-10-square-3 room-12-square-13)
 		(trans room-10-square-3 room-10-square-2)
 		(trans room-10-square-3 room-10-square-4)
 		(trans room-10-square-3 room-10-square-15)
 		(trans room-10-square-3 room-12-square-14)
 
  		 ;;; transitions for room-10-square-2
 		(trans room-10-square-2 room-10-square-3)
 		(trans room-10-square-2 room-12-square-13)
 	;;;(trans room-10-square-2 room-10-square-1)
 		(trans room-10-square-2 room-10-square-14)
 
  		 ;;; transitions for room-10-square-1
 	;;;(trans room-10-square-1 room-10-square-2)
 	;;;(trans room-10-square-1 room-10-square-12)
 		(trans room-10-square-1 room-10-square-13)
 
  		 ;;; transitions for room-10-square-9
 		(trans room-10-square-9 room-10-square-10)
 		(trans room-10-square-9 room-10-square-21)
 
  		 ;;; transitions for room-10-square-12
 	;;;(trans room-10-square-12 room-10-square-1)
 	;;;(trans room-10-square-12 room-10-square-11)
 	;;;(trans room-10-square-12 room-10-square-13)
 		(trans room-10-square-12 room-10-square-24)
 
  		 ;;; transitions for room-9-square-80
 		(trans room-9-square-80 room-10-square-10)
 
  		 ;;; transitions for room-10-square-11
 	;;;(trans room-10-square-11 room-10-square-12)
 	;;;(trans room-10-square-11 room-10-square-10)
 		(trans room-10-square-11 room-10-square-23)
 
  		 ;;; transitions for room-10-square-10
 		(trans room-10-square-10 room-10-square-9)
 		(trans room-10-square-10 room-9-square-80)
 	;;;(trans room-10-square-10 room-10-square-11)
 		(trans room-10-square-10 room-10-square-22)
 
  		 ;;; transitions for room-10-square-13
 	;;;(trans room-10-square-13 room-10-square-12)
 		(trans room-10-square-13 room-10-square-14)
 		(trans room-10-square-13 room-10-square-1)
 		(trans room-10-square-13 room-10-square-25)
 
  		 ;;; transitions for room-10-square-14
 		(trans room-10-square-14 room-10-square-13)
 		(trans room-10-square-14 room-10-square-15)
 		(trans room-10-square-14 room-10-square-2)
 		(trans room-10-square-14 room-10-square-26)
 
  		 ;;; transitions for room-10-square-15
 		(trans room-10-square-15 room-10-square-14)
 		(trans room-10-square-15 room-10-square-3)
 		(trans room-10-square-15 room-10-square-16)
 		(trans room-10-square-15 room-10-square-27)
 
  		 ;;; transitions for room-10-square-16
 		(trans room-10-square-16 room-10-square-17)
 		(trans room-10-square-16 room-10-square-4)
 		(trans room-10-square-16 room-10-square-15)
 		(trans room-10-square-16 room-10-square-28)
 
  		 ;;; transitions for room-10-square-17
 		(trans room-10-square-17 room-10-square-16)
 		(trans room-10-square-17 room-10-square-18)
 		(trans room-10-square-17 room-10-square-5)
 		(trans room-10-square-17 room-10-square-29)
 
  		 ;;; transitions for room-10-square-18
 		(trans room-10-square-18 room-10-square-17)
 		(trans room-10-square-18 room-10-square-19)
 		(trans room-10-square-18 room-10-square-6)
 		(trans room-10-square-18 room-10-square-30)
 
  		 ;;; transitions for room-10-square-19
 		(trans room-10-square-19 room-10-square-18)
 		(trans room-10-square-19 room-10-square-20)
 		(trans room-10-square-19 room-10-square-7)
 		(trans room-10-square-19 room-10-square-31)
 
  		 ;;; transitions for room-10-square-20
 		(trans room-10-square-20 room-10-square-19)
 		(trans room-10-square-20 room-10-square-8)
 		(trans room-10-square-20 room-10-square-32)
 
  		 ;;; transitions for room-10-square-21
 		(trans room-10-square-21 room-10-square-22)
 		(trans room-10-square-21 room-10-square-9)
 		(trans room-10-square-21 room-10-square-33)
 
  		 ;;; transitions for room-10-square-22
 		(trans room-10-square-22 room-10-square-21)
 		(trans room-10-square-22 room-10-square-23)
 		(trans room-10-square-22 room-10-square-10)
 		(trans room-10-square-22 room-10-square-33)
 
  		 ;;; transitions for room-10-square-23
 		(trans room-10-square-23 room-10-square-22)
 		(trans room-10-square-23 room-10-square-24)
 		(trans room-10-square-23 room-10-square-11)
 		(trans room-10-square-23 room-10-square-34)
 
  		 ;;; transitions for room-10-square-24
 		(trans room-10-square-24 room-10-square-23)
 		(trans room-10-square-24 room-10-square-25)
 		(trans room-10-square-24 room-10-square-12)
 		(trans room-10-square-24 room-10-square-35)
 
  		 ;;; transitions for room-10-square-25
 		(trans room-10-square-25 room-10-square-24)
 		(trans room-10-square-25 room-10-square-26)
 		(trans room-10-square-25 room-10-square-13)
 		(trans room-10-square-25 room-10-square-36)
 
  		 ;;; transitions for room-10-square-26
 		(trans room-10-square-26 room-10-square-25)
 		(trans room-10-square-26 room-10-square-27)
 		(trans room-10-square-26 room-10-square-14)
 		(trans room-10-square-26 room-10-square-37)
 
  		 ;;; transitions for room-10-square-27
 		(trans room-10-square-27 room-10-square-26)
 		(trans room-10-square-27 room-10-square-28)
 		(trans room-10-square-27 room-10-square-15)
 		(trans room-10-square-27 room-10-square-38)
 
  		 ;;; transitions for room-10-square-28
 		(trans room-10-square-28 room-10-square-27)
 		(trans room-10-square-28 room-10-square-29)
 		(trans room-10-square-28 room-10-square-16)
 		(trans room-10-square-28 room-10-square-39)
 
  		 ;;; transitions for room-10-square-29
 		(trans room-10-square-29 room-10-square-28)
 		(trans room-10-square-29 room-10-square-30)
 		(trans room-10-square-29 room-10-square-17)
 		(trans room-10-square-29 room-10-square-40)
 
  		 ;;; transitions for room-10-square-30
 		(trans room-10-square-30 room-10-square-29)
 		(trans room-10-square-30 room-10-square-31)
 		(trans room-10-square-30 room-10-square-18)
 		(trans room-10-square-30 room-10-square-41)
 
  		 ;;; transitions for room-10-square-31
 		(trans room-10-square-31 room-10-square-30)
 		(trans room-10-square-31 room-10-square-32)
 		(trans room-10-square-31 room-10-square-19)
 		(trans room-10-square-31 room-10-square-42)
 
  		 ;;; transitions for room-10-square-32
 		(trans room-10-square-32 room-10-square-31)
 		(trans room-10-square-32 room-10-square-20)
 		(trans room-10-square-32 room-10-square-43)
 
  		 ;;; transitions for room-10-square-33
 		(trans room-10-square-33 room-10-square-34)
 		(trans room-10-square-33 room-10-square-22)
 		(trans room-10-square-33 room-10-square-44)
 		(trans room-10-square-33 room-10-square-21)
 
  		 ;;; transitions for room-10-square-34
 		(trans room-10-square-34 room-10-square-33)
 		(trans room-10-square-34 room-10-square-35)
 		(trans room-10-square-34 room-10-square-23)
 		(trans room-10-square-34 room-10-square-45)
 
  		 ;;; transitions for room-10-square-35
 		(trans room-10-square-35 room-10-square-34)
 		(trans room-10-square-35 room-10-square-36)
 		(trans room-10-square-35 room-10-square-24)
 		(trans room-10-square-35 room-10-square-46)
 
  		 ;;; transitions for room-10-square-36
 		(trans room-10-square-36 room-10-square-35)
 		(trans room-10-square-36 room-10-square-37)
 		(trans room-10-square-36 room-10-square-25)
 		(trans room-10-square-36 room-10-square-47)
 
  		 ;;; transitions for room-10-square-37
 		(trans room-10-square-37 room-10-square-36)
 		(trans room-10-square-37 room-10-square-38)
 		(trans room-10-square-37 room-10-square-26)
 		(trans room-10-square-37 room-10-square-48)
 
  		 ;;; transitions for room-10-square-38
 		(trans room-10-square-38 room-10-square-37)
 		(trans room-10-square-38 room-10-square-39)
 		(trans room-10-square-38 room-10-square-27)
 		(trans room-10-square-38 room-10-square-49)
 
  		 ;;; transitions for room-10-square-39
 		(trans room-10-square-39 room-10-square-38)
 		(trans room-10-square-39 room-10-square-28)
 		(trans room-10-square-39 room-10-square-40)
 		(trans room-10-square-39 room-10-square-50)
 
  		 ;;; transitions for room-10-square-40
 		(trans room-10-square-40 room-10-square-39)
 		(trans room-10-square-40 room-10-square-41)
 		(trans room-10-square-40 room-10-square-29)
 		(trans room-10-square-40 room-10-square-51)
 
  		 ;;; transitions for room-10-square-41
 		(trans room-10-square-41 room-10-square-40)
 		(trans room-10-square-41 room-10-square-42)
 		(trans room-10-square-41 room-10-square-30)
 		(trans room-10-square-41 room-10-square-52)
 
  		 ;;; transitions for room-10-square-42
 		(trans room-10-square-42 room-10-square-41)
 		(trans room-10-square-42 room-10-square-43)
 		(trans room-10-square-42 room-10-square-31)
 		(trans room-10-square-42 room-10-square-53)
 
  		 ;;; transitions for room-10-square-43
 		(trans room-10-square-43 room-10-square-42)
 		(trans room-10-square-43 room-10-square-32)
 		(trans room-10-square-43 room-10-square-54)
 
  		 ;;; transitions for room-10-square-44
 		(trans room-10-square-44 room-10-square-45)
 		(trans room-10-square-44 room-10-square-33)
 		(trans room-10-square-44 room-10-square-55)
 
  		 ;;; transitions for room-10-square-45
 		(trans room-10-square-45 room-10-square-44)
 		(trans room-10-square-45 room-10-square-46)
 		(trans room-10-square-45 room-10-square-34)
 		(trans room-10-square-45 room-10-square-56)
 
  		 ;;; transitions for room-10-square-46
 		(trans room-10-square-46 room-10-square-45)
 		(trans room-10-square-46 room-10-square-47)
 		(trans room-10-square-46 room-10-square-35)
 		(trans room-10-square-46 room-10-square-57)
 
  		 ;;; transitions for room-10-square-47
 		(trans room-10-square-47 room-10-square-46)
 		(trans room-10-square-47 room-10-square-48)
 		(trans room-10-square-47 room-10-square-36)
 		(trans room-10-square-47 room-10-square-58)
 
  		 ;;; transitions for room-10-square-48
 		(trans room-10-square-48 room-10-square-47)
 		(trans room-10-square-48 room-10-square-49)
 		(trans room-10-square-48 room-10-square-37)
 		(trans room-10-square-48 room-10-square-59)
 
  		 ;;; transitions for room-10-square-49
 		(trans room-10-square-49 room-10-square-48)
 		(trans room-10-square-49 room-10-square-50)
 		(trans room-10-square-49 room-10-square-38)
 		(trans room-10-square-49 room-10-square-60)
 
  		 ;;; transitions for room-10-square-50
 		(trans room-10-square-50 room-10-square-49)
 		(trans room-10-square-50 room-10-square-39)
 		(trans room-10-square-50 room-10-square-51)
 		(trans room-10-square-50 room-10-square-61)
 
  		 ;;; transitions for room-10-square-51
 		(trans room-10-square-51 room-10-square-50)
 		(trans room-10-square-51 room-10-square-52)
 		(trans room-10-square-51 room-10-square-40)
 		(trans room-10-square-51 room-10-square-62)
 
  		 ;;; transitions for room-10-square-52
 		(trans room-10-square-52 room-10-square-51)
 		(trans room-10-square-52 room-10-square-53)
 		(trans room-10-square-52 room-10-square-41)
 		(trans room-10-square-52 room-10-square-63)
 
  		 ;;; transitions for room-10-square-53
 		(trans room-10-square-53 room-10-square-52)
 		(trans room-10-square-53 room-10-square-54)
 		(trans room-10-square-53 room-10-square-42)
 		(trans room-10-square-53 room-10-square-64)
 
  		 ;;; transitions for room-10-square-54
 		(trans room-10-square-54 room-10-square-53)
 		(trans room-10-square-54 room-10-square-43)
 		(trans room-10-square-54 room-10-square-65)
 
  		 ;;; transitions for room-10-square-55
 		(trans room-10-square-55 room-10-square-56)
 		(trans room-10-square-55 room-10-square-44)
 		(trans room-10-square-55 room-10-square-66)
 
  		 ;;; transitions for room-10-square-56
 		(trans room-10-square-56 room-10-square-55)
 		(trans room-10-square-56 room-10-square-57)
 		(trans room-10-square-56 room-10-square-45)
 		(trans room-10-square-56 room-10-square-67)
 
  		 ;;; transitions for room-10-square-57
 		(trans room-10-square-57 room-10-square-56)
 		(trans room-10-square-57 room-10-square-58)
 		(trans room-10-square-57 room-10-square-46)
 		(trans room-10-square-57 room-10-square-68)
 
  		 ;;; transitions for room-10-square-58
 		(trans room-10-square-58 room-10-square-57)
 		(trans room-10-square-58 room-10-square-59)
 		(trans room-10-square-58 room-10-square-47)
 		(trans room-10-square-58 room-10-square-69)
 
  		 ;;; transitions for room-10-square-59
 		(trans room-10-square-59 room-10-square-58)
 		(trans room-10-square-59 room-10-square-60)
 		(trans room-10-square-59 room-10-square-48)
 		(trans room-10-square-59 room-10-square-70)
 
  		 ;;; transitions for room-10-square-60
 		(trans room-10-square-60 room-10-square-59)
 		(trans room-10-square-60 room-10-square-61)
 		(trans room-10-square-60 room-10-square-49)
 		(trans room-10-square-60 room-10-square-71)
 
  		 ;;; transitions for room-10-square-61
 		(trans room-10-square-61 room-10-square-60)
 		(trans room-10-square-61 room-10-square-62)
 		(trans room-10-square-61 room-10-square-50)
 		(trans room-10-square-61 room-10-square-71)
 
  		 ;;; transitions for room-10-square-62
 		(trans room-10-square-62 room-10-square-61)
 		(trans room-10-square-62 room-10-square-63)
 		(trans room-10-square-62 room-10-square-51)
 		(trans room-10-square-62 room-10-square-72)
 
  		 ;;; transitions for room-10-square-63
 		(trans room-10-square-63 room-10-square-62)
 		(trans room-10-square-63 room-10-square-64)
 		(trans room-10-square-63 room-10-square-52)
 		(trans room-10-square-63 room-10-square-73)
 
  		 ;;; transitions for room-10-square-64
 		(trans room-10-square-64 room-10-square-63)
 		(trans room-10-square-64 room-10-square-65)
 		(trans room-10-square-64 room-10-square-53)
 		(trans room-10-square-64 room-10-square-74)
 
  		 ;;; transitions for room-10-square-65
 		(trans room-10-square-65 room-10-square-64)
 		(trans room-10-square-65 room-10-square-54)
 		(trans room-10-square-65 room-10-square-75)
 
  		 ;;; transitions for room-10-square-66
 		(trans room-10-square-66 room-10-square-67)
 		(trans room-10-square-66 room-10-square-55)
 		(trans room-10-square-66 room-10-square-76)
 
  		 ;;; transitions for room-10-square-67
 		(trans room-10-square-67 room-10-square-66)
 		(trans room-10-square-67 room-10-square-68)
 		(trans room-10-square-67 room-10-square-56)
 		(trans room-10-square-67 room-10-square-77)
 
  		 ;;; transitions for room-10-square-68
 		(trans room-10-square-68 room-10-square-67)
 		(trans room-10-square-68 room-10-square-69)
 		(trans room-10-square-68 room-10-square-57)
 		(trans room-10-square-68 room-10-square-78)
 
  		 ;;; transitions for room-10-square-69
 		(trans room-10-square-69 room-10-square-68)
 		(trans room-10-square-69 room-10-square-70)
 		(trans room-10-square-69 room-10-square-58)
 		(trans room-10-square-69 room-10-square-79)
 
  		 ;;; transitions for room-10-square-70
 		(trans room-10-square-70 room-10-square-69)
 		(trans room-10-square-70 room-10-square-71)
 		(trans room-10-square-70 room-10-square-59)
 		(trans room-10-square-70 room-10-square-80)
 
  		 ;;; transitions for room-10-square-71
 		(trans room-10-square-71 room-10-square-70)
 		(trans room-10-square-71 room-10-square-72)
 		(trans room-10-square-71 room-10-square-61)
 		(trans room-10-square-71 room-10-square-81)
 		(trans room-10-square-71 room-10-square-82)
 		(trans room-10-square-71 room-10-square-60)
 
  		 ;;; transitions for room-10-square-72
 		(trans room-10-square-72 room-10-square-71)
 		(trans room-10-square-72 room-10-square-73)
 		(trans room-10-square-72 room-10-square-62)
 		(trans room-10-square-72 room-10-square-83)
 
  		 ;;; transitions for room-10-square-73
 		(trans room-10-square-73 room-10-square-72)
 		(trans room-10-square-73 room-10-square-74)
 		(trans room-10-square-73 room-10-square-63)
 		(trans room-10-square-73 room-10-square-84)
 
  		 ;;; transitions for room-10-square-74
 		(trans room-10-square-74 room-10-square-73)
 		(trans room-10-square-74 room-10-square-75)
 		(trans room-10-square-74 room-10-square-64)
 		(trans room-10-square-74 room-10-square-85)
 
  		 ;;; transitions for room-10-square-75
 		(trans room-10-square-75 room-10-square-74)
 		(trans room-10-square-75 room-10-square-65)
 		(trans room-10-square-75 room-10-square-86)
 
  		 ;;; transitions for room-10-square-76
 		(trans room-10-square-76 room-10-square-66)
 		(trans room-10-square-76 room-10-square-77)
 		(trans room-10-square-76 room-9-square-80)
 
  		 ;;; transitions for room-10-square-77
 		(trans room-10-square-77 room-10-square-76)
 		(trans room-10-square-77 room-10-square-78)
 		(trans room-10-square-77 room-10-square-67)
 
  		 ;;; transitions for room-10-square-78
 		(trans room-10-square-78 room-10-square-77)
 		(trans room-10-square-78 room-10-square-79)
 		(trans room-10-square-78 room-10-square-68)
 
  		 ;;; transitions for room-10-square-79
 		(trans room-10-square-79 room-10-square-78)
 		(trans room-10-square-79 room-10-square-80)
 		(trans room-10-square-79 room-10-square-69)
 
  		 ;;; transitions for room-10-square-80
 		(trans room-10-square-80 room-10-square-79)
 		(trans room-10-square-80 room-10-square-81)
 		(trans room-10-square-80 room-10-square-70)
 		(trans room-10-square-80 room-10-square-87)
 
  		 ;;; transitions for room-10-square-81
 		(trans room-10-square-81 room-10-square-80)
 		(trans room-10-square-81 room-10-square-82)
 		(trans room-10-square-81 room-10-square-71)
 		(trans room-10-square-81 room-10-square-87)
 
  		 ;;; transitions for room-10-square-82
 		(trans room-10-square-82 room-10-square-81)
 		(trans room-10-square-82 room-10-square-71)
 		(trans room-10-square-82 room-10-square-83)
 		(trans room-10-square-82 room-10-square-87)
 
  		 ;;; transitions for room-10-square-83
 		(trans room-10-square-83 room-10-square-84)
 		(trans room-10-square-83 room-10-square-72)
 		(trans room-10-square-83 room-10-square-82)
 
  		 ;;; transitions for room-10-square-84
 		(trans room-10-square-84 room-10-square-83)
 		(trans room-10-square-84 room-10-square-85)
 		(trans room-10-square-84 room-10-square-73)
 
  		 ;;; transitions for room-10-square-85
 		(trans room-10-square-85 room-10-square-84)
 		(trans room-10-square-85 room-10-square-86)
 		(trans room-10-square-85 room-10-square-74)
 
  		 ;;; transitions for room-10-square-86
 		(trans room-10-square-86 room-10-square-85)
 		(trans room-10-square-86 room-10-square-75)
 
  		 ;;; transitions for room-10-square-87
 		(trans room-10-square-87 room-10-square-80)
 		(trans room-10-square-87 room-10-square-82)
 		(trans room-10-square-87 room-10-square-81)
 
 		(door-1-trans room-12-square-13 room-10-square-2)
 		(door-1-trans room-10-square-2 room-12-square-13)
 
 	 ;;; hard coded definitions
 		(is-robo cob4-1)
 		(is-user user)
 		(at the-cake room-10-square-10)
 		;;;(at the-cake room-10-square-11)
 		(at cob4-1 room-12-square-7)
 		(at user room-12-square-6)
 		;;;(at user room-10-square-48)
 		(neglected cob4-1)
 		;;;(nearby cob4-1 room-12-square-8)
 		(occupied room-10-square-2)
 		(occupied room-10-square-22)
 		(at the-box room-10-square-2)
 		(at the-box room-10-square-22)
 		;;;(door-1 room-12-square-13 room-10-square-2)
 	)
 
 
 	;;; goal definition
 	(:goal (and (have user the-cake)))
 	;;;(:goal (and (at the-cake room-12-square-7)))
 	;;;(:goal (and (have cob4-1 the-cake) (at cob4-1 room-12-square-7)))
 )
