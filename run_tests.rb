#!/usr/local/bin/ruby19
#!/usr/bin/env ruby
# File name: run2.rb
# Date:      Tue Oct 30 14:15:47 +0200 2012
# Author:    Miroslav Kulich

#require 'nats/client'
require "rubygems"
require "yaml"
require "trollop"
require "fileutils"
#require "eventmachine"
require "fileutils"
require 'timeout'

EXECUTABLE_NAME='main'

opts = Trollop::options do
  version "1.0 (c) 2012 Miroslav Kulich"
  banner <<-EOS
Runs experiments with various configurations

Usage:
       run.rb [options] 
where [options] are:
EOS
  opt :config, "YAML config file", :type => String, :default => "default.yaml", :short => "-c"
  opt :repeat, "Number of runs for each configuration", :default => 20, :short => "-r"
  opt :output, "Prefix of output path", :default => "results/all", :short => "-d"
  opt :port, "Port of player", :default => 6665, :short => "-p"
  opt :emport, "Port of eventmachine", :default => 10000, :short => "-e"
end

p opts


$output_path = opts[:output]
$player_port = opts[:port]
$eventmachine_port = opts[:emport]

par = YAML::load_file(opts[:config])

$params = par["var_params"];
$fixed_params = par["fixed_params"];
$configs = par["player_configs"];
$all_done = false
$status = {}

$repeat = opts[:repeat] #how many times each iteration will be executed
p "params: #{$params}"
p "number of repeats: #{$repeat}"
p "fixed params: #{$fixed_params}"
# $params = {
#   "mre-strategy" => [ "greedy", "hungarian", "mtspkmeans" ],
#  "cost-function" => [ "dist", "rot" ],
#   "rot-weight" => [ 0.1, 0.2, 0.5, 0.7 ]
#  "range" => [2,3]
# }
# 

#$fixed_params = {
#"cost-function" => "dist",
#"save-pic" => 0,  
#"image-quality" => 4
#}

  
#p params.keys
#method.each_pair { |method_name,method_index|

$pid = -1
$nats_data = {}
#$server = 0

#class InfoServer < EM::Connection

  ##
  ## EventMachine handlers
  ##

  #def post_init
    #puts "A client has connected..."
  #end

  #def unbind
    #puts "A client has left..."
  #end

  #def receive_data(data)
    #case data.strip 
      #when "imr_init_data" 
        #send_data "ahoj\n"
  #send_data $nats_data
      #when "imr_kill"
  #system("killall -9 player")
  #sleep(1)
  #send_data ("PLAYER KILLED\n")      
      #else
      #send_data "Unknown message\n"
    #end 
  #end
#end


def Process.descendant_processes(base)
  descendants = Hash.new{|ht,k| ht[k]=[k]}
  Hash[*`ps -eo pid,ppid`.scan(/\d+/).map{|x|x.to_i}].each{|pid,ppid|
    descendants[ppid] << descendants[pid]
  }
  descendants[base].flatten - [base]
end

def Process.kill_children(base)
puts "kill_children"  
  descendants = Process.descendant_processes(base)
puts "children: #{descendants}"
  descendants.each { |d| 
   puts ("killing #{d}")                  
   kill("TERM",d)                     
  }
   kill("TERM",base)                       
end


def build_output_path()
  $output_path += "/%playerconfig%"
  $params.each_key {|key|
    $output_path += "/%#{key}%"
  }
end

def one_run(par,var_params,conf)
#  if ($pid != -1)
#    Process.kill_children($pid)
#    $pid = -1
#  end

  
  #timer = EventMachine.add_timer(60*120) do
  #  p "timer: killing the application"
  #  system("killall -9 #{EXECUTABLE_NAME}")
  #  FileUtils.rmtree(var_params["robot-out-dir"])
  #end

  
  Timeout::timeout(10*60) {|t|

  ##  system("killall -9 player")
    sleep(1)
    
    $pid  = fork do
  #    Signal.trap("KILL") { puts "Ouch!"; exit }
      Dir.chdir("../test")
        FileUtils.rm_f(Dir.glob('writelog*'))
        command = "player -p #{$player_port} #{conf}.cfg" 
        p ("running player: #{command}")#  > /dev/null 2> /dev/null")
  #      system("#{command} > /dev/null 2> /dev/null")
        exec("#{command}")
  #      exec("#{command} > /dev/null 2> /dev/null")
    end
    p "player PID: #{$pid}"
    sleep(2)
    #puts "var:" + var_params + " " + $fixed_params
    act_params = par.merge(var_params).merge($fixed_params)
    command  = "./#{EXECUTABLE_NAME} " + to_params(act_params)
  #  command = "sleep 5"
    p "running program #{command}"
    $nats_data = to_nats_msg(act_params)
    if $server 
      $server.send_data($nats_data)
    end
    system(command + " 2&>1 > output.log")
    #FileUtils.copy("logs/tmre.log","logs/tmre-last.log")
    #if ( !File.exists?("#{var_params["output"]}/path.log") ) 
    #  FileUtils.rmtree(var_params["output"])
    #end
    puts "PPPPPP #{$pid}"
    if ($pid != -1)
      Process.kill_children($pid)
      $pid = -1
    end

    sleep(2)
    FileUtils.mkdir_p(var_params["output"])
    FileUtils.mv(Dir.glob('../test/writelog*'), var_params["output"])
    ##  system("killall -9 player")
  }
    
  #EventMachine.cancel_timer(timer)
rescue Timeout::Error
  p "timer: killing the application"
  system("killall -9 #{EXECUTABLE_NAME}")
  Process.kill_children($pid)
  FileUtils.rmtree(var_params["output"])
end

def to_params(par)
  result = ""
  par.each_pair{|key,val|
    result+=" --#{key} #{val}"
  }
  return result
end

def to_nats_msg(par)
  tmp = {"time" => Time.now, "status" => $status }
  return "#{YAML::dump(tmp.merge(par))}\n"
end



def get_output_path(par,conf)
  $output_path = $output_path.gsub(/%playerconfig%/,conf)
  $output_path.gsub(/%([\w-]*)%/) { |m|
    par[$1]
  }
end

def make_key(par,conf)
  result = conf
  par.each_value {|v|
    result += "|#{v}"
  }
  return result
end

def call(num, par,conf,iter)
  if (num < $params.keys.size)
    key = $params.keys[num]
    $params[key].each {|value|
      act = par
      par[key] = value
      call(num+1, act,conf,iter)
    }
  else
    path = get_output_path(par,conf)
    iter = Dir.exists?(path) ? Dir.entries(path).size-1 : 0 
    var_params = {
      "iter" => iter,
      "output" => "#{path}/#{Time.now.strftime("%Y%m%d-%H%M%S")}/"
    }
    
    if ( iter < $repeat )
      $status[make_key(par,conf)] = "#{iter}*"
      $all_done = false
      FileUtils.mkdir_p("#{var_params["output"]}pic");  
      one_run(par,var_params, conf)
      $status[make_key(par,conf)] = iter
    end
  end
end


def callforStatus(num,par,conf)
  if (num < $params.keys.size)
    key = $params.keys[num]
    $params[key].each {|value|
      act = par
      par[key] = value
      callforStatus(num+1,act,conf)
    }
  else
    path = get_output_path(par,conf)
    iter = Dir.exists?(path) ? Dir.entries(path).size-1 : 0 
    $status[make_key(par,conf)] = iter
  end
end


def getStatus(output)
  $configs.each { |conf| 
    $output_path = output     
    build_output_path
    callforStatus(0,{},conf)
  }
  $nats_data = to_nats_msg({})
  p $status
end




##################################333
# MAIN
##################################333

getStatus(opts[:output])     


#EventMachine.run {
##  EventMachine.start_server("0.0.0.0", $eventmachine_port, InfoServer) do |s| ##
##    $server = s
##  end

  #Thread.new do
    $configs.each { |conf|
      $output_path = opts[:output]     
      build_output_path
      $all_done = false
      while !$all_done do            
      $all_done = true
        call(0,{},conf,0)
      end
    }
    #EventMachine.stop
  #end
#}
##system("killall -9 player")

puts "done"
