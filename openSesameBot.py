import discord
from discord.ext import commands
from discord import app_commands
import os

#========================= Constants =========================
#will fill in after to avoid secret being in repo
botToken = '';
#grab token from local env variable
botToken = os.getenv("DISCORD_TOKEN");

GUILD_ID = discord.Object(id=1490450047085838446);



 
#========================= Init =========================


#========================= Class =========================
class Client(commands.Bot):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        
    async def on_message(self, message):
        if message.author == self.user:
            return
        if message.content.startswith('!sesame'):
            await message.channel.send(f'Command Recieved: {message.author} executed: {message.content}');

    
 
    
    
    
#========================= Main stuffs =========================

intents = discord.Intents.default();
intents.message_content = True;
client = Client(command_prefix = "!sesame", intents = intents);






@client.tree.command(name="door", description="Controls Door", guild = GUILD_ID)
async def printer(interaction: discord.Interaction):
    embed = discord.Embed(title = "Door State", description = "TestState");
    await interaction.response.send_message(embed = embed);












#run bot
client.run(botToken);
