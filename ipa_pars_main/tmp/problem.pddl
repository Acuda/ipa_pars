(define (problem cob-test-problem-01)
 	(:domain cob-test-domain-01)
 	(:objects 
  		 ;;; available objects 
  
  		 ;;; fixed locations 
<<<<<<< HEAD
 		 room-1 room-2 room-3 - room
  
 		 ;;; dynamically added trash-bins
 		 trash-bin-1 trash-bin-2 trash-bin-3 - trash-bin
  		
 		 ;;; dynamically added dirt-locations
 		 dirt-1 dirt-2 dirt-3 - dirt
 		
 		 ;;; robot
 		 cob4-1 - robot
 		 vacuum-cleaner - tool
 	)
=======
 		 room-12-square-1 room-12-square-2 
 		 room-12-square-3 room-12-square-10 room-12-square-7 
 		 room-12-square-6 room-12-square-4
 		 room-12-square-8 room-12-square-9 room-12-square-5 room-11-square-3 
 		 room-12-square-11 room-12-square-12 room-12-square-13 room-12-square-14 
 		 room-10-square-8 room-10-square-7 room-10-square-6 room-10-square-5 
 		 room-10-square-4 room-10-square-3 room-10-square-2 room-10-square-1 
 		 room-10-square-9 room-10-square-12 room-10-square-11 
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
 		 room-10-square-84 room-10-square-85 room-10-square-86 room-10-square-87 - room 
  
 		 ;;; fixed things for interaction
 		 arm-left - gripper
 		 arm-right - gripper
 		 the-boss - user
 
  		 ;;; movable things 
 		 the-cake - phys-obj 
 		 cob4-1 - robot
 		 the-box-1 the-box-2 the-box-3 the-box-4 the-box-5 the-box-6 the-box-7  - phys-obj
 		)
>>>>>>> github-ce/indigo_dev
 
 
 	(:init  
  		;;; transitions for room-12-square-2
 		(trans room-12-square-2 room-12-square-3)
 		(trans room-12-square-2 room-12-square-7)
 		(trans room-12-square-2 room-12-square-8)
 
  		 ;;; transitions for room-12-square-3
 		(trans room-12-square-3 room-12-square-2)
 		(trans room-12-square-3 room-12-square-10)
 		(trans room-12-square-3 room-12-square-9)
 		(trans room-12-square-3 room-12-square-1)
 
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
 		(trans room-12-square-6 room-12-square-4)
 		(trans room-12-square-6 room-12-square-5)
 		(trans room-12-square-6 room-12-square-11)
 
  		 ;;; transitions for room-12-square-4
 		(trans room-12-square-4 room-12-square-6)
 
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
 		(trans room-12-square-5 room-12-square-6)
 
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
 
<<<<<<< HEAD
  		 ;;; transitions for room-2
 		(trans room-2 room-1)
 		(trans room-2 room-3)
=======
  		 ;;; transitions for room-10-square-35
 		(trans room-10-square-35 room-10-square-34)
 		(trans room-10-square-35 room-10-square-36)
 		(trans room-10-square-35 room-10-square-24)
 		(trans room-10-square-35 room-10-square-46)
>>>>>>> github-ce/indigo_dev
 
  		 ;;; transitions for room-10-square-36
 		(trans room-10-square-36 room-10-square-35)
 		(trans room-10-square-36 room-10-square-37)
 		(trans room-10-square-36 room-10-square-25)
 		(trans room-10-square-36 room-10-square-47)
 
<<<<<<< HEAD
 		;;; dynamically added locations
 		(at trash-bin-1 room-1)
 		(at trash-bin-2 room-1)
 		(at trash-bin-3 room-3)
 		(at dirt-1 room-1)
 		(at dirt-2 room-1)
 		(at dirt-3 room-3)
 
 		;;; inital robot location
 		(at cob4-1 room-2)
=======
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
 
 	 ;;; hard coded definitions
 		;;;(is-robo cob4-1)
 		;;;(is-user user)
 		(at the-cake room-10-square-10)
 		;;;(at the-cake room-12-square-8)
 		;;;(at the-cake room-10-square-11)
 		(at cob4-1 room-12-square-7)
 		(at the-boss room-12-square-6)
 		;;;(at user room-10-square-48)
 		(neglected cob4-1)
 		;;;(nearby cob4-1 room-12-square-8)
 		(occupied room-10-square-2)
 		(occupied room-10-square-22)
 		(at the-box-1 room-10-square-2)
 		(at the-box-2 room-10-square-22)
 		(at the-box-3 room-10-square-13)
 		(at the-box-4 room-10-square-36)
 		(at the-box-5 room-10-square-27)
 		(at the-box-6 room-10-square-3)
 		(at the-box-7 room-10-square-9)
 
 		;;; gripper
 		(which-gripper arm-left)
 		(which-gripper arm-right)
 		(gripper-free arm-left)
 		(gripper-free arm-right)
 
 		
 		;;;(door-1 room-12-square-13 room-10-square-2)
>>>>>>> github-ce/indigo_dev
 	)
 
 
 	;;; goal definition
<<<<<<< HEAD
 	(:goal (and (forall (?r - room) (inspected ?r))
 		    (forall (?t - trash-bin) (cleared ?t))
 		    (forall (?d - dirt) (and (cleaned ?d) (verify-cleaning-success ?d)))
 		    (process-results cob4-1)
 		    
 		)
 	)
=======
 	(:goal (and (have the-boss the-cake)))
 	;;;(:goal (and (at the-cake room-12-square-7)))
 	;;;(:goal (and (have cob4-1 the-cake) (at cob4-1 room-12-square-7)))
>>>>>>> github-ce/indigo_dev
 )
